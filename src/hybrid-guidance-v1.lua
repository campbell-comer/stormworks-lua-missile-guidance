-- Hybrid Missile Guidance (SARH-ARH)
--
-- Semi-Active Radar Homing / Active Radar Homing hybrid guidance system
-- with Proportional Navigation (PN) and PID flight controllers.
--
-- Engagement modes:
--   SARH:      Follow datalink waypoint coordinates
--   ARH:       Active radar homing using onboard radar
--   RTLP:      Recursive Time-Limited Pursuit (track loss recovery)
--   Pit Bull:  Force active radar homing based on proximity
--
-- INPUTS (numbers)
--   1..3   Missile GPS position XYZ
--   4..5   Radar bearing X/Y (turns)
--   6..7   Front/rear radar distance
--   8..9   Front/rear radar signal strength
--   10..13 Missile orientation (pitch, heading, right pitch, right heading)
--   14..16 Datalink target position XYZ
--   17..19 Datalink target velocity XYZ
--
-- INPUTS (bools)
--   1  Fire   2  Uncage   3  Hit detect   4  Use hybrid mode
--
-- OUTPUTS (numbers)
--   1..2   Horizontal/vertical control commands
--   3..5   Radar data passthrough
--   27..29 Target position XYZ
--   30..32 Missile position XYZ
--
-- OUTPUTS (bools)
--   1  Active   2  Radar detecting   3  Fired   4  RTLP engaged
--
-- PROPERTIES
--   "P", "I", "D"          PID gains
--   "1".."6"               GPS and altimeter offsets
--   "7"                    Radar back offset
--   "8"                    Mass filter threshold
--   "Missile Lifetime"     Max flight time (seconds)
--   "Safety Time"          Delay before guidance engages (seconds)

------------------------
-- I/O and math aliases
------------------------
m = math
IN = input.getNumber
ON = output.setNumber
gB = input.getBool
OB = output.setBool
PIN = property.getNumber
pgB = property.getBool
sin, cos, tan, atan, abs, asin, acos = m.sin, m.cos, m.tan, m.atan, m.abs, m.asin, m.acos
min = m.min
max = m.max
pi = m.pi
pi2 = pi * 2

------------------------
-- Vector math and utilities
------------------------
delta = {}
relAxes = {}

function removeNaNfromX(x)
	return (x ~= x or abs(x) == m.huge) and 0 or x
end

function clamp(a, b, c)
    return min(max(a, b), c)
end

function Vector(x,y,z)
	return {x=x,y=y,z=z}
end

nilVec = Vector(0, 0, 0)

tgtPosRaw = nilVec

function sgn(x)
	return  x < 0 and -1 or 1
end

function subVecs(A, B)
	return addVecs(A, vecScaleN(B, -1))
end

function addVecs(A, B)
	return Vector(A.x + B.x, A.y + B.y, A.z + B.z)
end

function divVecs(A, R)
	return vecScaleN(A, 1 / R)
end

function vecScale(A, B)
	return Vector(A.x * B.x, A.y * B.y, A.z * B.z)
end

function vecScaleN(A, n)
	return Vector(A.x * n, A.y * n, A.z * n)
end

function vecNormalize(S)
	return divVecs(S, vecLength(S))
end

function vecLength(S)
	return vecDot(S,S)^.5
end

function vecDot(a, b)
	return a.x * b.x + a.y * b.y + a.z * b.z
end

function vecCross(a, b)
	return Vector(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x)
end

------------------------
-- Coordinate conversions
------------------------
-- Spherical (yaw, pitch, distance) to Cartesian vector
function eulerToVector(x,y,s)
	s = s or 1
	scalar = cos(y) * s
	return Vector(sin(x) * scalar, cos(x) * scalar, sin(y) * s)
end

-- Cartesian vector to spherical (yaw, pitch) angles
function vectorToEuler(vec)
	return Vector(acos(vecNormalize(Vector(vec.x, vec.y, 0)).y) * sgn(vec.x), asin(vecNormalize(vec).z), 0)
end

------------------------
-- Reference frame transforms
------------------------
function relToLocal(x, relangle)
	return Vector(vecDot(x, relangle.r), vecDot(x, relangle.fwd), vecDot(x, relangle.up))
end

function relToGlobal(vec, relAxes)
	return addVecs(addVecs(vecScaleN(relAxes.r, vec.x), vecScaleN(relAxes.fwd, vec.y)), vecScaleN(relAxes.up, vec.z))
end

-- Build orthonormal basis from heading and pitch sensor readings
function relativeAxes(HDG_forward, tilt_forward, HDG_right, tilt_right)
	forward_directional = eulerToVector(-HDG_forward, tilt_forward, 1)
	right_directional = eulerToVector(-HDG_right, tilt_right, 1)
	vertical_directional = vecCross(right_directional, forward_directional)

	index = {
		fwd = forward_directional,
		r = right_directional,
		up = vertical_directional
	}

	return index
end

------------------------
-- Radar and guidance helpers
------------------------
-- Convert radar bearing angles and distances to absolute world position
function radarToAbsPos(x,y,disX,disY,pos,relangles)
	hh=sin(x)*disX
	hv=sin(y)*disY
	k1=cos(y)*disY
	l=((k1)^2-(hh)^2)^.5
	relVector=addVecs(relToGlobal(addVecs(Vector(hh, l, hv), Vector(0, radarBackOffset, 0)),relangles),pos)
	return relVector
end

-- Sigmoid activation for variable navigation gain
function sigmoid(max_value,x)
	e = math.exp(1)
	return max_value/1+e^((-x*15)+e^2)
end

-- Per-tick delta for velocity estimation
function deltaVector(vec, id)
	delta[id] = delta[id] or nilVec
	out = subVecs(vec, delta[id])
	delta[id] = vec
	return out
end

-- Tangential guidance: compute missile velocity aligned with LOS plane
function calcTang(gtc, dtc, dmc)
	los = vecNormalize(gtc)
	missileTang = projectVectorOnPlane(dtc, los)
	missileOrtLength = abs(max(vecLength(dmc), 0.5) ^ 2 - vecLength(missileTang) ^ 2) ^ .5
	missileOrt = vecScaleN(los, missileOrtLength)
	return addVecs(missileTang, missileOrt)
end

function linePlaneIntersection(linePos, lineVec, planeVec, planeNorm)
	return addVecs(
		linePos,
		vecScaleN(lineVec, (vecDot(planeNorm, planeVec) - vecDot(planeNorm, linePos)) / vecDot(planeNorm, lineVec))
	)
end

function projectVectorOnPlane(vec, normal)
	return linePlaneIntersection(vec, normal, nilVec, normal)
end

function vecReflect(vec,norm,k)
	return subVecs(vecScaleN(projectVectorOnPlane(vec,vecNormalize(norm)),k or 2),vec)
end

-- Apply GPS and altimeter offsets to raw position
function fixPos(x, relAxes)
	gpsoffset, altoffset =
		vecScale(relToGlobal(gpsOffset, relAxes), Vector(1, 1, 0)),
		vecScale(relToGlobal(altOffset, relAxes), Vector(0, 0, 1))
	return addVecs(addVecs(x, gpsoffset), altoffset)
end

------------------------
-- PID controller
------------------------
pids = {}
function PID(set, process, p, i, d, id, on, imin, imax)
	pids[id] = pids[id] or {0, 0}
	error = set - removeNaNfromX(process)
	der = error - pids[id][1]
	int = max(min(error + pids[id][2], (imax or 1) / i), (imin or -1) / i)
	pids[id][1] = error
	pids[id][2] = int
	if not on then
		pids[id] = {0, 0}
	end
	return on and (p * error + i * int + d * der) or 0
end

--[[maxEntries = 130
movements = {}
movementSum = nilVec
velSum = 0
delay = 0]]

--[[function motionAverage(movementVec,time,freeze)
	start = time > 1
    table.insert(movements,{vec = movementVec})
    if #movements > maxEntries then
        table.remove(movements,1)
    end
    movementSum = nilVec
    for e,a in ipairs(movements) do
        movementSum = addVecs(movementSum, a.vec)
    end
    avgPath = divVecs(movementSum, #movements)
	if not freeze then
    avgVel = time > maxEntries and deltaVector(avgPath,7) or vecScaleN(deltaVector(avgPath,7), start and 2 or 1)
    avgAcc = deltaVector(avgVel,8)
	fixedAcc = start and avgAcc or nilVec
	end
    return addVecs(addVecs(avgPath,vecScaleN(avgVel,time + delay)),vecScaleN(fixedAcc,.5 * (time+delay)^2))
end]]

------------------------
-- Recursive Time-Limited Pursuit (RTLP)
-- Fallback target prediction when radar lock is lost
------------------------
count = 0
function RTLP(pos,velocity,acceleration,time,mc)
	if time%230 < 130 and time > 30 then
		predictedPos = addVecs(vecScaleN(velocity,vecLength(subVecs(pos,mc))),mc)
		predictedDirection = addVecs(velocity,vecScaleN(acceleration,.5 * time^2))
		direction = vecNormalize(predictedDirection)
		reflectedVector = vecReflect(direction,subVecs(pos,mc))
		count = 0
		return addVecs(reflectedVector,mc)
	else
		count = count + 1
		return count < 60 and addVecs(addVecs(pos,vecScaleN(velocity,time)),vecScaleN(acceleration,.5 * time^2)) or pos
	end
end

------------------------
-- Properties
------------------------
P,I,D = PIN("P"),PIN("I"),PIN("D")
gpsOffset = Vector(PIN("1"), PIN("2"), PIN("3"))
altOffset = Vector(PIN("4"), PIN("5"), PIN("6"))
radarBackOffset = PIN("7")
massFilter = PIN("8")
lifeTime = PIN("Missile Lifetime")
delay = PIN("Safety Time")
--useHybrid = pgB("Use SARH-ARH Hybrid")

------------------------
-- State initialization
------------------------
stc = nilVec
tgtPos = nilVec
wtc = nilVec
stv = nilVec
sta = nilVec
maxVel = 0
k = 0
lrelAxes={r=nilVec,fwd=nilVec,up=nilVec}
timeSinceLastDetection = 0

------------------------
-- Main loop
------------------------
function onTick()
	radarX = IN(4) * pi2
	radarY = IN(5) * pi2
	distF = IN(6)
	distR = IN(7)
	strengthF = IN(8)
	strengthR = IN(9)
	pitch = IN(10) * pi2
	HDG = IN(11) * pi2
	pitchRight = IN(12) * pi2
	HDGright = IN(13) * pi2
	fire = gB(1)
	uncage = gB(2)
	useHybrid = gB(4)

	wtc = Vector(IN(14), IN(15), IN(16))
	wtcVelocity = Vector(IN(17),IN(18),IN(19))

	relAxes = relativeAxes(HDG, pitch, HDGright, pitchRight)
	pos = fixPos(Vector(IN(1), IN(2), IN(3)), relAxes)

	if fire then
		k = min(k + 1/60, lifeTime)
	else
		k = 0
	end
	-- Disable logic when missile is not active, hit detected, or lifetime expired
	if not fire and not uncage or k>=lifeTime or gB(3) then
		pitBull = false
		if not did then
			for i=1,32 do
				ON(i,0)
				OB(i,false)
				did=i>31
			end
		end
		return
	else
		did=false
	end
	targetMass = ((strengthF + strengthR) / 2) * ((distF + distR) / 2)
	detectRadar = distF > 0 and distR > 0 and targetMass > massFilter and (targetMass <= 2499 or targetMass >= 2501)
	wtcCheck = (not (wtc.x == 0 and wtc.y == 0)) and useHybrid

    fireOrUncage = fire or uncage

	-- Pit Bull activation: force active radar homing when in range
	if wtcCheck and vecLength(subVecs(wtc, pos)) < 4000 and fireOrUncage and detectRadar or fireOrUncage and detectRadar and not wtcCheck then
		pitBull = true
	elseif not detectRadar and wtcCheck and vecLength(wtcVelocity) > 0 then
		pitBull = false
	end


	tgtPosRad = radarToAbsPos(radarX, radarY, distF, distR,pos,relAxes)
	tgtVelRad = deltaVector(tgtPosRad,1)
	tgtAccRad = deltaVector(tgtVelRad,2)

	--RTLP = motionAverage(stc,timeSinceLastDetection,engageRTLP)

	-- Target source selection based on radar lock and hybrid mode state
	if detectRadar and fireOrUncage and not wtcCheck or wtcCheck and detectRadar and pitBull then
		engageRTLP = false
		tgtPos = tgtPosRad
		stc = tgtPos
		if distF > 15 then
			stv = tgtVelRad
			sta = tgtAccRad
		end
		timeSinceLastDetection = 0
	elseif wtcCheck and fire and not pitBull then
		engageRTLP = false
		tgtPos = wtc
	elseif not wtcCheck and fire and not detectRadar and (stc.x == 0 and stc.y == 0) then
		tgtPos = nilVec
	else
		engageRTLP = true
		timeSinceLastDetection = timeSinceLastDetection + 1
		tgtPos = RTLP(stc,stv,sta,timeSinceLastDetection,pos)
		tgtPos.z = max(tgtPos.z,20)
	end


	detect = not (tgtPos.x == 0 and tgtPos.y == 0)
	tgtDeltaPos = subVecs(tgtPos, pos)
	tgtVel, misVel = deltaVector(tgtPos, 3), deltaVector(pos, 4)
	maxVel = k > .1 and max(maxVel,vecLength(misVel)) or 0

	tgtAcc = deltaVector(tgtVel, 5)
	-- Variable navigation gain (sigmoid-scaled by range and speed ratio)
	varNav = sigmoid(P,max(clamp(3000 / vecLength(tgtDeltaPos), 0, 1) * (vecLength(misVel) / maxVel), .5))
	leadingVec = calcTang(tgtDeltaPos, tgtVel, misVel)

	relativeVelAngles = vectorToEuler(relToLocal(misVel, relAxes))
	leadingVecAngles = subVecs(vectorToEuler(relToLocal(leadingVec,relAxes)),relativeVelAngles)

    armVelAlign = fire and k < delay or fire and not detect
    armPID = fire and detect

	-- PN guidance: align missile with leading vector via PID
	horizontalAlign, verticalAlign = PID(0,-leadingVecAngles.x,varNav,I,D,1,armPID,-.1,.1),PID(0,-leadingVecAngles.y,varNav,I,D,2,armPID,-.1,.1)
	rotationSpeed = vectorToEuler(relToLocal(relAxes.fwd,lrelAxes))
	cmd = Vector(horizontalAlign, verticalAlign, 0)
	 -- Velocity alignment / heading lock
	velAlign = Vector(PID(0,rotationSpeed.x,.9,.8,.6,3,armVelAlign,-1,1),PID(0,rotationSpeed.y,.9,.8,.6,4,armVelAlign,-1,1),0)
	-- Flight mode: use PN guidance if locked, velocity alignment otherwise
	autopilot = (k >= delay and detect) and cmd or armVelAlign and velAlign or nilVec

	ON(1,autopilot.x)
	ON(2,autopilot.y)
	ON(3,radarX/pi2)
	ON(4,radarY/pi2)
	ON(5,distF)
	OB(1,true)
	OB(2,detectRadar)
	OB(3,fire)
	OB(4,engageRTLP)
	ON(27,tgtPos.x)
	ON(28,tgtPos.y)
	ON(29,tgtPos.z)
	ON(30,pos.x)
	ON(31,pos.y)
	ON(32,pos.z)
	lrelAxes=relAxes
end
