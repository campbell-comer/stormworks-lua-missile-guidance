-- [Legacy] Tangential Missile Guidance
--
-- Original proportional navigation / tangential guidance
-- implementation. Superseded by hybrid-guidance-v1.lua.

targetVel = {}
pos = {}
tgtPos = {}
relAxes = {}
started = false
startDistance = 0
neededAltitude = 0
gN = input.getNumber
sN = output.setNumber
sin = math.sin
cos = math.cos
atan = math.atan
asin = math.asin
sqrt = math.sqrt
pi = math.pi
pi2 = math.pi * 2
gB = input.getBool
max = math.max
abs = math.abs
pi = math.pi
pgN = property.getNumber
t1 = 0
t2 = 0
t3 = 0
t4 = 0
t5 = 0
t6 = 0
reset = 0
RadarFOV = 0.125

targetX_raw = nil
targetY_raw = nil
targetZ_raw = nil

previoustick_X = 0
previoustick_Y = 0
previoustick_Z = 0

function lerp(a,b,t)
return a * (1-t) + b * t
end

function vecScaleN(A, n)
	return Vector(A.x * n, A.y * n, A.z * n)
end

function sigmoid(max_value,x) --input from 0 to 1
local e = math.exp(1)
x_axis = x*15
return max_value/1+e^(-x+e^2)
end

function vector_combine(b, c, e)
    a = sqrt(b ^ 2 + c ^ 2)
    return sqrt(a ^ 2 + e ^ 2)
end

function nilVec()
    return Vector(0, 0, 0)
end

function clamp(a, h, i)
    return math.min(math.max(a, h), i)
end

function dot(a, b)
    return a.x * b.x + a.y * b.y + a.z * b.z
end

function pid(y, z, d)
    return {
        p = y,
        i = z,
        d = d,
        E = 0,
        D = 0,
        I = 0,
        run = function(E, F, G)
            local H, D, A
            H = F - G
            D = H - E.E
            A = math.abs(D - E.D)
            E.E = H
            E.D = D
            E.I = A < H and E.I + H * E.i or E.I * 0.5
            return H * E.p + (A < H and E.I or 0) + D * E.d
        end
    }
end

function dist(J, K, L, M)
    local N = L - J
    local O = M - K
    d = sqrt(N ^ 2 + O ^ 2)
    return d
end
Vector = function(b, c, e)
    return {x = b, y = c, z = e}
end
Plane = function(Q)
    return {
        pos = nilVec(),
        rot = Vector(Q.x, Q.y, Q.z)
    }
end

function vecScale(A, B)
	return Vector(A.x * B.x, A.y * B.y, A.z * B.z)
end

function subVecs(A, B)
    return Vector(A.x - B.x, A.y - B.y, A.z - B.z)
end

function addVecs(A, B)
    return Vector(A.x + B.x, A.y + B.y, A.z + B.z)
end

function divVecs(A, R)
    return Vector(A.x / R, A.y / R, A.z / R)
end

function multVecs(A, R)
    return Vector(A.x * R, A.y * R, A.z * R)
end

function vecLength(S)
    return abs((S.x * S.x + S.y * S.y + S.z * S.z) ^ .5)
end

function vector_invert(vec)
return Vector(-vec.x,-vec.y,-vec.z)
end

function SphericaltoCartesian(yawangle, pitchangle, distance)
    local alt = (sin(pitchangle) * distance)
    local relativeXY = (cos(pitchangle) * distance)
    local relativeX = (sin(yawangle) * relativeXY)
    local relativeY = (cos(yawangle) * relativeXY)
    return relativeX, relativeY, alt
end

function normalize(S)
    return divVecs(S, vecLength(S))
end

function projectVec(S, T)
    return linePlaneIntersection(S, T.rot, T.pos, T.rot)
end
function reflect(vector, normal)
    projPlane = Plane(normal)
    tang = projectVec(vector, projPlane)
    ort = projectVectorOnLine(vector, normal)
    return subVecs(tang, ort)
end
function projectVectorOnLine(vector, lineVector)
    dir = normalize(lineVector)
    return multVecs(dir, dot(vector, lineVector) / vecLength(lineVector))
end

function linePlaneIntersection(W, _, a0, a1)
    A = a1.x
    B = a1.y
    C = a1.z
    D = 0
    t = -(A * W.x + B * W.y + C * W.z + D) / (A * _.x + B * _.y + C * _.z)
    res = addVecs(W, multVecs(_, t))
    return res
end

function toGlobal(vec, relAxes)
	return addVecs(addVecs(vecScaleN(relAxes.right, vec.x), vecScaleN(relAxes.forward, vec.y)), vecScaleN(relAxes.up, vec.z))
end

function crossProduct(a, b)
    return Vector(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x)
end

function angleDiff(a, a2)
    return (a2 - a + pi * 3) % pi2 - pi
end

P = pgN("P")
I = pgN("I")
D = pgN("D")
pid1 = pid(P, I, D)
pid2 = pid(P, I, D)

function onStart()
    startDistance = CurrentDistance
end

function removeNaNfromX(x)
  return (x ~= x or math.abs(x) == math.huge) and 0 or x
end

function vector_undefined(vec)
if vec.x == 0 and vec.y == 0 and vec.z == 0 then return true end
end

ticks = pgN("Ticks To Extrapolate")
guidance = pgN("Guidance Method")

GPSoffset = Vector(pgN("GPS offset X"), pgN("GPS offset Y"), pgN("GPS offset Z"))
ALToffset = Vector(pgN("Alt offset X"), pgN("Alt offset Y"), pgN("Alt offset Z"))

function onTick()
--if safety is on or lifetime is up disable the script
if gN(12) ~= 1 or gN(13) == 1 then return end

    onOff = gB(1)
    targetX_raw = gN(1)
    targetY_raw = gN(2)
    targetZ_raw = gN(3)
    X = gN(5)
    Y = gN(6)
    Z = gN(7)-1
    compass = gN(8) * pi2
    tilt_right = gN(9) * pi2
    pitch = gN(10) * pi2
    HDG_left = gN(11) * pi2
    tilt_left = -tilt_right
   
	posRaw = Vector(X,Y,Z)
	tgtPos = Vector(targetX_raw,targetY_raw,targetZ_raw)
   
      df_x, df_y, df_z = SphericaltoCartesian(-compass, pitch, 1)
    directional_forward = Vector(df_x, df_y, df_z)

    dr_x, dr_y, dr_z = SphericaltoCartesian(-HDG_left, tilt_left, 1)
    directional_left = Vector(dr_x, dr_y, dr_z)

    directional_up = crossProduct(directional_forward, directional_left) 
   
    relAxes = {
    	forward = directional_forward,
    	right = vector_invert(directional_left),
    	up = directional_up
    }
   
    GPSGlobalOffset = vecScale(toGlobal(GPSoffset, relAxes), Vector(-1, -1, 0))
	ALTGlobalOffset = vecScale(toGlobal(ALToffset, relAxes), Vector(0, 0, -1))
   
   	missilePos = addVecs(addVecs(posRaw, GPSGlobalOffset), ALTGlobalOffset)
   
    tvx = tgtPos.x - t1
    t1 = tgtPos.x
    tvy = tgtPos.y - t2
    t2 = tgtPos.y
    tvz = tgtPos.z - t3
    t3 = tgtPos.z
    mx = X - t4
    t4 = X
    my = Y - t5
    t5 = Y
    mz = Z - t6
    t6 = Z

    targetX_extr = tgtPos.x + (tvx * ticks)
    targetY_extr = tgtPos.y + (tvy * ticks)
    targetZ_extr = tgtPos.z + (tvz * ticks)

	if vector_undefined(Vector(targetX_extr,targetY_extr,targetZ_extr)) == true then

	targetX = previoustick_X == nil and 0 or previoustick_X
	targetY = previoustick_Y == nil and 0 or previoustick_Y
	targetZ = previoustick_Z == nil and 0 or previoustick_Z
		else
	targetX = targetX_extr
	targetY = targetY_extr
	targetZ = targetZ_extr

end

    missileVel = Vector(mx, my, mz)
    speed = vecLength(missileVel)
    targetVel = Vector(tvx, tvy, tvz)
    tgtPos = Vector(targetX, targetY, targetZ)
    targetDelta = subVecs(tgtPos, missilePos)
    tgtspeed = vector_combine(tvx, tvy, tvx) * 60
    tgtDir = normalize(targetDelta)
    LOSPerpendicularPlane = Plane(tgtDir)
    targetTang = projectVec(targetVel, LOSPerpendicularPlane)
    missileTang = targetTang

    if speed ^ 2 - vecLength(missileTang) ^ 2 < 0 then
        sN(1, 0)
        sN(2, 0)
        return
    end

    missileOrtLen = abs(speed ^ 2 - vecLength(missileTang) ^ 2) ^ .5
    missileOrt = multVecs(tgtDir, missileOrtLen)
    missileDir = addVecs(missileTang, missileOrt)
    leadvector_normal = addVecs(multVecs(normalize(targetDelta), vecLength(missileOrt)), targetTang)
   
    if guidance == 1 then
		leadvector = leadvector_normal
			else if guidance == 2 then
		leadvector = vector_invert(reflect(Vector(0,0,-1),leadvector_normal))
		end
	end
   
    leadsetpoint = normalize(leadvector)

    relativeVector =
        Vector(
        dot(leadsetpoint, directional_left),
        dot(leadsetpoint, directional_forward),
        dot(leadsetpoint, directional_up)
    )

--lead vec calculations
    leadVectorHDG = atan(relativeVector.x, relativeVector.y)
    leadVectorptch =atan(relativeVector.z, sqrt(relativeVector.x ^ 2 + relativeVector.y ^ 2))
    CurrentDistance = dist(X, Y, targetX, targetY)

    if started == false and onOff then
        onStart()
        started = true
    end

    sN(1, pid1:run(0, -leadVectorHDG))
    sN(2, pid2:run(0, leadVectorptch))
    sN(3, targetX)
    sN(4, targetY)
    sN(5, targetZ)
    sN(6, leadvector.x)
    sN(7, leadvector.y)
    sN(8, leadvector.z)
    sN(9, leadVectorHDG)
   
    if vector_undefined(Vector(targetX_extr,targetY_extr,targetZ_extr)) == false then
 previoustick_X = targetX_extr
 previoustick_Y = targetY_extr
 previoustick_Z = targetZ_extr
    end
end