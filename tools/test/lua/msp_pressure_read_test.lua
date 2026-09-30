-- msp_pressure_read_test.lua - Mud System Physics tyre pressure as a compaction source.
-- Guards SoilCompactionModel.readMSPPressureKPa and its place in computeForVehicle:
-- MSP's TirePressureSystem is read from MSP's own mod environment (_G[modName]) and
-- converted like the VTP bar value; anything unexpected falls back to geometry.
--!load: src/utils/Logger.lua, src/config/Constants.lua, src/SoilCompactionModel.lua

local gp = SoilConstants.COMPACTION.GROUND_PRESSURE
local function kpa(bar) return bar * gp.BAR_TO_KPA + gp.CONTACT_OFFSET_KPA end

local MSP = "FS25_MudSystemPhysics"

-- A fake MSP: the getter returns what the vehicle carries, like MSP's per-wheel average.
local function makeTPS()
    return {
        enabled = true,
        getVehicleWheelPressureAverage = function(self, vehicle)
            if vehicle.throws then error("boom") end
            return vehicle.bar, vehicle.bar, vehicle.wheelCount
        end,
    }
end

local function installMSP(tps)
    g_modIsLoaded = { FS25_SomeOtherMod = true, [MSP] = true }
    _G[MSP] = { TirePressureSystem = tps }
    g_currentMission = {}   -- a new mission forces the name lookup to run again
end

local function wheel(z, tireType)
    return { physics = { wheelShapeWidth = 0.65, radius = 0.9, positionZ = z, tireType = tireType } }
end

local function vehicle(bar, opts)
    opts = opts or {}
    local v = {
        bar = bar,
        wheelCount = opts.wheelCount or 4,
        throws = opts.throws,
        spec_wheels = { wheels = opts.wheels or { wheel(1.5), wheel(1.5), wheel(-1.2), wheel(-1.2) } },
        getTotalMass = function() return 12 end,
    }
    return v
end

-- 1. Found: the vehicle's own average bar becomes contact kPa, like VTP.
local tps = makeTPS()
installMSP(tps)
T.near("1.0 bar reads as field kPa", SoilCompactionModel.readMSPPressureKPa(vehicle(1.0)), kpa(1.0))
T.near("2.4 bar reads as road kPa", SoilCompactionModel.readMSPPressureKPa(vehicle(2.4)), kpa(2.4))

-- 2. MSP's own switch off: values would be frozen, so no read.
tps.enabled = false
T.eq("tire pressure switched off -> nil", SoilCompactionModel.readMSPPressureKPa(vehicle(1.0)), nil)
tps.enabled = true

-- 3. Bad getter results fall back.
T.eq("0 wheels -> nil (getter still returns 2.4)", SoilCompactionModel.readMSPPressureKPa(vehicle(2.4, { wheelCount = 0 })), nil)
T.eq("getter throws -> nil", SoilCompactionModel.readMSPPressureKPa(vehicle(1.0, { throws = true })), nil)
T.eq("non-number bar -> nil", SoilCompactionModel.readMSPPressureKPa(vehicle("x")), nil)
T.eq("zero bar -> nil", SoilCompactionModel.readMSPPressureKPa(vehicle(0)), nil)
T.eq("no spec_wheels -> nil", SoilCompactionModel.readMSPPressureKPa({ bar = 1.0, wheelCount = 4 }), nil)

-- 4. MSP missing, or its environment without TirePressureSystem.
g_modIsLoaded = { FS25_SomeOtherMod = true }
g_currentMission = {}
T.eq("MSP not loaded -> nil", SoilCompactionModel.readMSPPressureKPa(vehicle(1.0)), nil)
g_modIsLoaded = { [MSP] = true }
_G[MSP] = { SomethingElse = {} }
g_currentMission = {}
T.eq("no TirePressureSystem in MSP env -> nil", SoilCompactionModel.readMSPPressureKPa(vehicle(1.0)), nil)

-- 5. A renamed zip still matches the name patterns.
g_modIsLoaded = { FS25_Mud_System_Physics_v2 = true }
_G["FS25_Mud_System_Physics_v2"] = { TirePressureSystem = makeTPS() }
g_currentMission = {}
T.near("renamed MSP zip still found", SoilCompactionModel.readMSPPressureKPa(vehicle(1.0)), kpa(1.0))

-- 6. Fully tracked vehicles skip MSP; mixed machines keep it.
installMSP(makeTPS())
local w1, w2, w3, w4 = wheel(1.5), wheel(1.5), wheel(-1.2), wheel(-1.2)
local tracked = vehicle(1.0, { wheels = { w1, w2, w3, w4 } })
tracked.spec_crawlers = { crawlers = {
    { wheel = w1, wheels = { { wheel = w1 }, { wheel = w3 } } },
    { wheel = w2, wheels = { { wheel = w2 }, { wheel = w4 } } },
} }
T.eq("fully tracked (Crawlers links every wheel) -> nil", SoilCompactionModel.readMSPPressureKPa(tracked), nil)

local m1, m2, m3, m4 = wheel(1.5), wheel(1.5), wheel(-1.2), wheel(-1.2)
local mixed = vehicle(1.0, { wheels = { m1, m2, m3, m4 } })
mixed.spec_crawlers = { crawlers = { { wheel = m3, wheels = { { wheel = m3 } } }, { wheel = m4, wheels = { { wheel = m4 } } } } }
T.near("mixed tracks + tyres keeps MSP", SoilCompactionModel.readMSPPressureKPa(mixed), kpa(1.0))

WheelsUtil = { getTireType = function(name) if string.upper(name) == "CRAWLER" then return 7 end return nil end }
local byTireType = vehicle(1.0, { wheels = { wheel(1.5, 7), wheel(1.5, 7), wheel(-1.2, 7), wheel(-1.2, 7) } })
T.eq("fully tracked by crawler tire type -> nil", SoilCompactionModel.readMSPPressureKPa(byTireType), nil)
local oneTyre = vehicle(1.0, { wheels = { wheel(1.5, 7), wheel(1.5, 1), wheel(-1.2, 7), wheel(-1.2, 7) } })
T.near("one non-crawler tyre keeps MSP", SoilCompactionModel.readMSPPressureKPa(oneTyre), kpa(1.0))
WheelsUtil = nil

-- 7. Order in computeForVehicle: VTP where active, then MSP, then geometry.
installMSP(makeTPS())
local v = vehicle(1.0)
local p, _, source = SoilCompactionModel.computeForVehicle(v)
T.eq("MSP present, no VTP -> source msp", source, "msp")
T.near("MSP present, no VTP -> MSP kPa", p, kpa(1.0))

v.spec_variableTirePressure = {}
v.vtpGetDashboardPressureBar = function() return 2.0 end
p, _, source = SoilCompactionModel.computeForVehicle(v)
T.eq("VTP active wins over MSP", source, "vtp")
T.near("VTP active -> VTP kPa", p, kpa(2.0))

v.vtpGetDashboardPressureBar = function() return 0 end
p, _, source = SoilCompactionModel.computeForVehicle(v)
T.eq("VTP inactive (0) falls through to MSP", source, "msp")

g_modIsLoaded = {}
g_currentMission = {}
p, _, source = SoilCompactionModel.computeForVehicle(vehicle(1.0))
T.eq("no VTP, no MSP -> geometry", source, "geometry")
T.ok("geometry still returns a pressure", type(p) == "number" and p > 0)

-- 8. Fully tracked vehicles get the fixed track contact pressure, with or without MSP,
--    instead of the geometry estimate that reads track rollers as tiny tyres.
local function trackedVehicle()
    local a, b, c, d = wheel(1.5), wheel(1.5), wheel(-1.2), wheel(-1.2)
    local t = vehicle(1.0, { wheels = { a, b, c, d } })
    t.spec_crawlers = { crawlers = {
        { wheel = a, wheels = { { wheel = a }, { wheel = c } } },
        { wheel = b, wheels = { { wheel = b }, { wheel = d } } },
    } }
    return t
end
local trackKPa = gp.TRACK_CONTACT_KPA
T.ok("track figure sits inside the sourced 60-110 kPa range", trackKPa >= 60 and trackKPa <= 110)

p, _, source = SoilCompactionModel.computeForVehicle(trackedVehicle())
T.eq("tracked, no MSP -> source tracks", source, "tracks")
T.near("tracked, no MSP -> track kPa", p, trackKPa)

installMSP(makeTPS())
p, _, source = SoilCompactionModel.computeForVehicle(trackedVehicle())
T.eq("tracked, MSP present -> still tracks, not msp", source, "tracks")
T.near("tracked, MSP present -> track kPa", p, trackKPa)

local tv = trackedVehicle()
tv.spec_variableTirePressure = {}
tv.vtpGetDashboardPressureBar = function() return 1.2 end
p, _, source = SoilCompactionModel.computeForVehicle(tv)
T.eq("VTP active on a tracked vehicle still wins", source, "vtp")

local _, axleT = SoilCompactionModel.computeForVehicle(trackedVehicle())
T.near("tracked subsoil stays on load (12 t over 2 axles)", axleT, 6)

T.summary()
