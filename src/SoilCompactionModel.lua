-- =====================================================================================
-- SoilCompactionModel.lua
-- -------------------------------------------------------------------------------------
-- Ground-pressure compaction model, grounded in extension agronomy (Penn State,
-- Missouri) rather than raw vehicle mass. Compaction per pass is two independent terms,
-- scaled by a soil-moisture multiplier:
--
--   SURFACE  ∝ ground contact pressure ≈ tyre inflation pressure. Wide / flotation /
--             aired-down tyres spread the load → low pressure → little surface packing.
--   SUBSOIL  ∝ axle load (independent of tyres). ~10 t/axle damages subsoil; <5 t/axle
--             does not. The permanent-damage term that a big tyre cannot avoid.
--   MOISTURE multiplier: wet soil compacts far worse ("hydraulic ram"); dry resists.
--
-- Variable Tire Pressure (HotShotPepper, ModHub) integration:
--   When VTP is installed it exposes a live, transition-interpolated effective pressure
--   in bar via vehicle:vtpGetDashboardPressureBar(). Surface contact pressure ≈ tyre
--   inflation pressure (PSU), so we read that bar value DIRECTLY as our surface pressure
--   - airing down to FIELD mode automatically lowers compaction. With VTP absent we
--   approximate contact pressure from each tyre's own loaded size instead (#1057).
--
-- Mud System Physics (BMP, ModHub) integration:
--   MSP keeps its own per-wheel tyre pressure (0.8-2.8 bar). It has no public API, so
--   we read its TirePressureSystem from MSP's own mod environment (_G[modName]) and
--   convert the vehicle's own average pressure exactly as the VTP bar value. Order per
--   vehicle: VTP where active, then MSP, then the geometry estimate.
--
-- Fully tracked vehicles: the geometry estimate reads track rollers as tiny tyres and
--   always gave the maximum surface points. They get a fixed TRACK_CONTACT_KPA instead
--   (measured soil stress, see Constants) and never take the MSP read (VTP leaves
--   crawler wheels out too). The subsoil term stays on load.
--
-- The vehicle/VTP reads live here; the scoring math (scorePoints / advanceWetness) is
-- pure and unit-tested under tools/test.
-- =====================================================================================

SoilCompactionModel = SoilCompactionModel or {}

local function clamp(v, lo, hi)
    if v < lo then return lo end
    if v > hi then return hi end
    return v
end

local GRAVITY = 9.81

-- -------------------------------------------------------------------------------------
-- PURE: compaction points for a single pass.
--   pressureKPa : surface contact pressure (nil → no surface term)
--   axleLoadT   : tonnes per axle  (nil → no subsoil term)
--   wetness01   : 0 (bone dry) .. 1 (saturated)
--   rateMult    : tuningCompactionRate multiplier (0 disables all build-up)
-- -------------------------------------------------------------------------------------
function SoilCompactionModel.scorePoints(pressureKPa, axleLoadT, wetness01, rateMult)
    local cp = SoilConstants and SoilConstants.COMPACTION
    if not cp then return 0 end

    rateMult = rateMult or 1.0
    if rateMult <= 0 then return 0 end
    wetness01 = clamp(wetness01 or 0, 0, 1)

    -- Surface term: contact pressure between FLOOR and REF maps linearly to 0..SURFACE_MAX.
    local surface = 0
    local gp = cp.GROUND_PRESSURE
    if gp and pressureKPa and pressureKPa > gp.FLOOR_KPA then
        local span = math.max(1e-6, gp.REF_KPA - gp.FLOOR_KPA)
        surface = gp.SURFACE_MAX * clamp((pressureKPa - gp.FLOOR_KPA) / span, 0, 1)
    end

    -- Subsoil term: axle load between FLOOR_T and REF_T maps linearly to 0..SUBSOIL_MAX.
    local subsoil = 0
    local al = cp.AXLE_LOAD
    if al and axleLoadT and axleLoadT > al.FLOOR_T then
        local span = math.max(1e-6, al.REF_T - al.FLOOR_T)
        subsoil = al.SUBSOIL_MAX * clamp((axleLoadT - al.FLOOR_T) / span, 0, 1)
    end

    -- Moisture multiplier scales the combined damage.
    local moist = 1.0
    local m = cp.MOISTURE
    if m then
        moist = m.DRY_MULT + (m.WET_MULT - m.DRY_MULT) * wetness01
    end

    return (surface + subsoil) * moist * rateMult
end

-- -------------------------------------------------------------------------------------
-- PURE: advance a decaying soil-wetness value (0..1).
--   Raining pins it to 1; otherwise it fades to 0 over MOISTURE.DECAY_HOURS.
-- -------------------------------------------------------------------------------------
function SoilCompactionModel.advanceWetness(prev, dtHours, isRaining)
    if isRaining then return 1.0 end
    prev = prev or 0
    local cp = SoilConstants and SoilConstants.COMPACTION
    local decayHours = (cp and cp.MOISTURE and cp.MOISTURE.DECAY_HOURS) or 12.0
    local dec = (dtHours or 0) / math.max(0.01, decayHours)
    return math.max(0, prev - dec)
end

-- -------------------------------------------------------------------------------------
-- VTP read: live effective contact pressure in kPa, or nil when VTP is not driving
-- this vehicle. Detection is by the registered getter + spec presence, never a guess.
-- -------------------------------------------------------------------------------------
function SoilCompactionModel.readVTPPressureKPa(vehicle)
    if vehicle == nil then return nil end
    if type(vehicle.vtpGetDashboardPressureBar) ~= "function" then return nil end
    if vehicle.spec_variableTirePressure == nil then return nil end
    local ok, bar = pcall(vehicle.vtpGetDashboardPressureBar, vehicle)
    if not ok or type(bar) ~= "number" or bar <= 0 then return nil end
    local gp = SoilConstants.COMPACTION.GROUND_PRESSURE
    return bar * gp.BAR_TO_KPA + gp.CONTACT_OFFSET_KPA
end

-- -------------------------------------------------------------------------------------
-- Mud System Physics read. MSP's TirePressureSystem is a global in MSP's own mod
-- environment, reached through _G[modName] (the route MSP itself uses for Use Your
-- Tyres). The mod name is resolved once per mission; the table is re-read on every
-- call, and anything unexpected returns nil so the caller falls back to geometry.
-- -------------------------------------------------------------------------------------
SoilCompactionModel.MSP_NAME_PATTERNS = { "mudsystem", "mud_system", "mudphysic" }

local mspLookup = { mission = nil, modName = nil }

local function scanMSPModName()
    if g_modIsLoaded == nil then return false end
    for modName, loaded in pairs(g_modIsLoaded) do
        if loaded then
            local lower = string.lower(tostring(modName))
            for _, pattern in ipairs(SoilCompactionModel.MSP_NAME_PATTERNS) do
                if lower:find(pattern, 1, true) then
                    local env = _G[modName]
                    if type(env) == "table" and type(rawget(env, "TirePressureSystem")) == "table" then
                        return modName
                    end
                end
            end
        end
    end
    return false
end

local function resolveMSPModName()
    local mission = g_currentMission
    if mission == nil or mspLookup.mission ~= mission then
        mspLookup.modName = scanMSPModName()
        if mission ~= nil then
            mspLookup.mission = mission
            if mspLookup.modName then
                SoilLogger.info("Compaction: Mud System Physics tire pressure found (%s)", tostring(mspLookup.modName))
            else
                SoilLogger.info("Compaction: Mud System Physics tire pressure not found - wheel-size estimate only")
            end
        end
    end
    return mspLookup.modName
end

local function getMSPTirePressureSystem()
    local modName = resolveMSPModName()
    if not modName then return nil end
    local env = _G[modName]
    if type(env) ~= "table" then return nil end
    local tps = rawget(env, "TirePressureSystem")
    if type(tps) ~= "table" or type(tps.getVehicleWheelPressureAverage) ~= "function" then return nil end
    return tps
end

-- True when every wheel of the vehicle belongs to a track: linked by the Crawlers
-- specialization, or carrying the "crawler" tire type (the two checks VTP uses).
-- Mixed machines (tracks plus tyres) are not fully tracked and keep the MSP read.
local function isFullyTracked(vehicle)
    local wheels = vehicle.spec_wheels and vehicle.spec_wheels.wheels
    if type(wheels) ~= "table" then return false end

    local trackWheels = {}
    local crawlerSpec = vehicle.spec_crawlers
    if crawlerSpec ~= nil and type(crawlerSpec.crawlers) == "table" then
        for _, crawler in pairs(crawlerSpec.crawlers) do
            if type(crawler) == "table" then
                if crawler.wheel ~= nil then trackWheels[crawler.wheel] = true end
                if type(crawler.wheels) == "table" then
                    for _, entry in pairs(crawler.wheels) do
                        if type(entry) == "table" and entry.wheel ~= nil then
                            trackWheels[entry.wheel] = true
                        end
                    end
                end
            end
        end
    end

    local crawlerTireType = nil
    if WheelsUtil ~= nil and WheelsUtil.getTireType ~= nil then
        local ok, tireType = pcall(WheelsUtil.getTireType, "crawler")
        if ok then crawlerTireType = tireType end
    end

    local anyWheel = false
    for _, wheel in pairs(wheels) do
        if type(wheel) == "table" then
            anyWheel = true
            local phys = wheel.physics
            local isCrawlerTire = crawlerTireType ~= nil and phys ~= nil and phys.tireType == crawlerTireType
            if not trackWheels[wheel] and not isCrawlerTire then
                return false
            end
        end
    end
    return anyWheel
end

-- Live average pressure of THIS vehicle's own wheels as contact pressure in kPa, or nil
-- when MSP is missing, its tire pressure is switched off (values would be frozen), the
-- vehicle is fully tracked, or the read returns anything unexpected.
function SoilCompactionModel.readMSPPressureKPa(vehicle)
    if vehicle == nil or vehicle.spec_wheels == nil then return nil end
    local tps = getMSPTirePressureSystem()
    if tps == nil or tps.enabled == false then return nil end
    if isFullyTracked(vehicle) then return nil end

    local ok, currentBar, _, wheelCount = pcall(tps.getVehicleWheelPressureAverage, tps, vehicle)
    if not ok or type(currentBar) ~= "number" or currentBar <= 0 then return nil end
    if type(wheelCount) ~= "number" or wheelCount <= 0 then return nil end
    local gp = SoilConstants.COMPACTION.GROUND_PRESSURE
    return currentBar * gp.BAR_TO_KPA + gp.CONTACT_OFFSET_KPA
end

-- -------------------------------------------------------------------------------------
-- Geometry fallback: contact pressure ≈ (own weight) / (Σ tyre contact patch), where a
-- patch ≈ width × (radius × CONTACT_LENGTH_FACTOR). The radius is the tyre's loaded size,
-- radiusOriginal, which the engine sets once from the wheel XML (WheelPhysics:loadFromXML
-- at game 1.24.0.0); the live radius is only the fallback when that is missing. A mod that
-- shrinks the live radius at runtime (airing down, sinking, wear) no longer shrinks the
-- patch and raises this estimate, the opposite of the design above (#1057). Airing down
-- lowers compaction only through a pressure read: VTP's or MSP's (above).
-- -------------------------------------------------------------------------------------
function SoilCompactionModel.readGeometryPressureKPa(vehicle, massT)
    if vehicle == nil or not massT or massT <= 0 then return nil end
    local spec = vehicle.spec_wheels
    if spec == nil or spec.wheels == nil then return nil end

    local gp = SoilConstants.COMPACTION.GROUND_PRESSURE
    local sumAreaM2 = 0
    for _, wheel in pairs(spec.wheels) do
        local phys = wheel.physics
        if phys then
            local width  = phys.wheelShapeWidth
            local radius = phys.radiusOriginal or phys.radius
            if width and radius and width > 0 and radius > 0 then
                sumAreaM2 = sumAreaM2 + width * (radius * gp.CONTACT_LENGTH_FACTOR)
            end
        end
    end
    if sumAreaM2 <= 0 then return nil end

    local loadN = massT * 1000.0 * GRAVITY
    return (loadN / sumAreaM2) / 1000.0  -- Pa → kPa
end

-- Count distinct axles by bucketing wheels on their local Z position (handles duals,
-- where 4 wheels share one axle). Falls back to a wheel-count estimate.
local function countAxles(vehicle)
    local cp = SoilConstants.COMPACTION
    local perAxle = (cp.AXLE_LOAD and cp.AXLE_LOAD.WHEELS_PER_AXLE) or 2
    local spec = vehicle.spec_wheels
    if spec == nil or spec.wheels == nil then return 2 end

    local buckets, axleCount, wheelCount = {}, 0, 0
    for _, wheel in pairs(spec.wheels) do
        wheelCount = wheelCount + 1
        local phys = wheel.physics
        local z = phys and phys.positionZ
        if z then
            local key = math.floor(z * 2 + 0.5)  -- ~0.5 m buckets
            if not buckets[key] then
                buckets[key] = true
                axleCount = axleCount + 1
            end
        end
    end
    if axleCount > 0 then return axleCount end
    if wheelCount > 0 then return math.max(1, math.ceil(wheelCount / perAxle)) end
    return 2
end

local function readMass(vehicle, onlyThis)
    local ok, m = pcall(function() return vehicle:getTotalMass(onlyThis) end)
    if ok and type(m) == "number" and m > 0 then return m end
    return nil
end

-- -------------------------------------------------------------------------------------
-- Resolve (pressureKPa, axleLoadT, source) for a vehicle. Everything is self-consistent
-- to THIS vehicle (its own mass over its own wheels). Returns nil if mass is unreadable.
--   source ∈ "vtp" | "tracks" | "msp" | "geometry"
-- -------------------------------------------------------------------------------------
function SoilCompactionModel.computeForVehicle(vehicle)
    if vehicle == nil then return nil end
    local massT = readMass(vehicle, true) or readMass(vehicle, false)
    if not massT then return nil end

    local source = "geometry"
    local pressureKPa = SoilCompactionModel.readVTPPressureKPa(vehicle)
    if pressureKPa then
        source = "vtp"
    elseif isFullyTracked(vehicle) then
        pressureKPa = SoilConstants.COMPACTION.GROUND_PRESSURE.TRACK_CONTACT_KPA
        source = "tracks"
    else
        pressureKPa = SoilCompactionModel.readMSPPressureKPa(vehicle)
        if pressureKPa then
            source = "msp"
        else
            pressureKPa = SoilCompactionModel.readGeometryPressureKPa(vehicle, massT)
        end
    end

    local axleLoadT = massT / countAxles(vehicle)
    return pressureKPa, axleLoadT, source
end

-- -------------------------------------------------------------------------------------
-- Convenience: full points value for a vehicle in one call (used by hooks).
-- Returns points (>=0) and the source string for logging.
-- -------------------------------------------------------------------------------------
function SoilCompactionModel.pointsForVehicle(vehicle, wetness01, rateMult)
    local pressureKPa, axleLoadT, source = SoilCompactionModel.computeForVehicle(vehicle)
    if pressureKPa == nil and axleLoadT == nil then return 0, source end
    return SoilCompactionModel.scorePoints(pressureKPa, axleLoadT, wetness01, rateMult), source
end

SoilLogger.info("SoilCompactionModel loaded")
