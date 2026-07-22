-- OrionPublic Wireshark Dissector 
-- Trillium Engineering, Inc. 2026
-- This dissector is designed to parse OrionPublic packets as defined in the Orion Public ICD. 
-- It extracts the packet type ID, packet enumeration, data length, and system time from the packet header,
--  and provides helper functions to parse and display the contents of various packet types in a human-readable format.
-- [supported Packets]
--	• APTINA_SETTINGS
--	• BOARD
--	• CAMERAS
--	• CAMERA_STATE
--	• CAMERA_SWITCH
--	• CLEVIS_VERSION
--	• CMD
--	• CROWN_MODE
--	• CROWN_VERSION
--	• DIAGNOSTICS
--	• EXT_HEADING_DATA
--	• Orion Fault
--	• FLIR_SETTINGS
--	• GeoidUndulation
--	• GEOLOCATIE Telemetry
--	• GEOPOINT_CMD
--	• GeoTrackerCommand
--	• GPS_DATA
--	• Orion Hitachi Settings
--	• INS_Options
--	• INS_Quality
--	• KLV User data
--	• Laser Command
--	• Laser States
--  • Lens Control
--	• Limits
--	• Network Diagnostics
--	• Network Settings
--	• Network Video
--	• Omnivis settings
--	• Payload Version
--	• Performance 
--	• Positions
--	• Product
--	• PRF Detect
--	• Range Data
--	• Reset Source
--	• Retract Command
--	• Retract Status
--	• Retract Version
--	• Sierra Settings
--	• Sensor Data
--	• Software Diagnostics
--	• Software Version
--	• Stare Ack
--	• Startup CMD
--	• Tle Command
--	• Track Command
--	• Tracker Version
--	• Uart Config
--	• User Data
--	• Vibration
--	• Video Options
--	• VideoRecordClock
--	• VideoRecordCmd
--	• VideoRecordStatus
--	• Stare Start
-- [Packets That will be added in official release but are not yet optimized in the dissector]
--	• Orion Path
--	• TLE Detects

-- [Filtering and Displaying Packets]
-- To use, place in the "User plugins" folder. OrionPublic packets can be filtered
--   as follows:
--
--  * OrionPublic:
--      This filter shows all frames which contain OrionPublic packets
--  * OrionPublic.id == 212 (or 0xD4):
--      Use this form to show only frames which contain OrionPublic packets with numerical ID
--  * OrionPublic.length == 7:
--      Use this form to show only frames which contain OrionPublic packets with
--      a data payload length of 7
--  * OrionPublic.packet == ORION_PKT_SONY_SETTINGS: see [OrionPktType_t] in the ICD
--      Use this form to show frames that contain packets matching the packet enumeration
-- Declare the OrionPublic Protocol

OrionPublic = Proto("OrionPublic", "Orion Protocol")

-- the new fields that contain the extracted data
orion_id = ProtoField.new("Packet Type ID", "OrionPublic.id", ftypes.UINT8)
orion_packet = ProtoField.new("Packet Type ENUM", "OrionPublic.packet", ftypes.STRING)
orion_len = ProtoField.new("Data Length", "OrionPublic.length", ftypes.UINT8)
orion_systime = ProtoField.new("System Time", "OrionPublic.time", ftypes.UINT32)

-- register the new fields into our protocol
OrionPublic.fields = { orion_id, orion_packet, orion_len, orion_systime }

--------------------- Helper functions --------------------
-- The checkIsProtocolPacket function return true/false if the data should be parsed as a OrionPublic packet
function checkIsOrionPublicPacket(buffer)
    return  (buffer(0,2):uint() == 53261)
end

-- The checkProtocolPacketIsValid function recieves the current buffer, size, index and id to determine if the packet can be parsed
-- Return a string to be displayed in the dissector, or an empty string if packet can be parsed and is valid
function checkOrionPublicPacketIsValid(subtree, buffer, id, name, size, index)
    local isInvalid = ""
    if (id >= 103 and id <= 111) or id == 120 or id == 121 then -- Orion Camera Settings packet
        if size == 1 then
            isInvalid = name .. "[R][Camera: " .. buffer(index, 1):uint() .. "]"
            subtree:add(buffer(index,1), "Requesting " .. name .. "[" .. buffer(index, 1):uint() .. "] Settings from Gimbal")
        elseif size == 0 then
            isInvalid = name .. "[R]"
            subtree:add("Requesting [" .. name .. "] Packet from Gimbal")
        end
    elseif size == 0 then
        isInvalid = name .. "[R]"
        subtree:add("Requesting [" .. name .. "] Packet from Gimbal")
    end
    return isInvalid
end

-- The getOrionPublicHeaderLength function returns the size of the packet header for the OrionPublic protocol
function getOrionPublicHeaderLength()
    return 6
end
-- The getOrionPublicPacketIdentifier function will attempt to decode the fields created above. Lua wireshark dissector expects an id, size, and a data section
function getOrionPublicPacketIdentifier(buffer)
    local id = buffer(2,1):uint()
	local size = buffer(3,1):uint()
	local data = buffer(4,size)
    return id, size, data
end

function addUnknownOrionPublicPacket(subtree, data, id, size )
    local info = string.format("Unknown packet ID 0x%x", id)
    subtree = subtree:add(OrionPublic, data, info)
    subtree:add(data, "Data: " .. data)
    return info
end

function makeSubtreeOrionPublic(subtree, buffer, name, id, size)
	subtree = subtree:add(OrionPublic, buffer, name)
	subtree:add(orion_id, id)
	subtree:add(orion_packet, get_OrionPktType_t_string(id))
	subtree:add(orion_len, size)
	return subtree
end

function makeStructureSubtreeOrionPublic(subtree, buffer, name, size)
    subtree = subtree:add(OrionPublic, buffer, name)
	subtree:add(orion_len, size)
	return subtree
end

function registerOrionPublicDissectorTable()
    DissectorTable.get("udp.port"):add(8745, OrionPublic)
    DissectorTable.get("udp.port"):add(8746, OrionPublic)
    DissectorTable.get("tcp.port"):add(8747, OrionPublic)
    DissectorTable.get("udp.port"):add(8748, OrionPublic)
end

-- helper values
datum_semiMajorAxis = 6378137.0
datum_flattening = (1.0 / 298.257223563)
datum_semiMinorAxis = (datum_semiMajorAxis * (1.0 - datum_flattening))
datum_eSquared = (1.0 - (datum_semiMinorAxis * datum_semiMinorAxis) / (datum_semiMajorAxis * datum_semiMajorAxis))
datum_eSecondSquared = ((datum_semiMajorAxis * datum_semiMajorAxis) / (datum_semiMinorAxis * datum_semiMinorAxis) - 1.0)

-- Input: current subtree, name of value, value, floating point radians
function floatToString(max_precesion, value)
    return  string.gsub(string.format(("%." .. max_precesion .. "f"), value), "%.?0*$", "")
end
--------------------------------- Display Methods -----------------------------------------------------

function displayLosECEF(posLatRad, posLonRad, posAlt, subtree, name, data, index, size, scaler)
    local losECEF = subtree:add(data(index, size), name)
    local gimbal_ecef = lla_to_ecef(posLatRad, posLonRad, posAlt)

    local x = data(index,2):int()
    local y = data(index+2,2):int()
    local z = data(index+4,2):int()

    lla = ecef_to_lla(x + gimbal_ecef[0], y + gimbal_ecef[1], z + gimbal_ecef[2])

    losECEF:add(data(index,2), "losECEFX: " .. x .. " m")
    losECEF:add(data(index+2,2), "losECEFY: " .. y .. " m")
    losECEF:add(data(index+4,2), "losECEFZ: " .. z .. " m")
    displayRadians(data(index,6), losECEF, "Latitude", lla[0])
    displayRadians(data(index,6), losECEF, "Longitude", lla[1])
    losECEF:add(data(index,6), string.format("Altitude: %.1f", lla[2]) .. " m")
    losECEF:add(data(index,6), string.format("Slant Range: %.1f", math.sqrt(x*x+y*y+z*z)))
    return index + size
end

function displayRadians(data, subtree, name, value)
    rads = floatToString(8, value)
    degs = floatToString(8, ((value*180)/math.pi))
	subtree:add(data, name .. ": " .. rads .. " rad (" .. degs .. " deg)")
end

function displayDegrees(data, subtree, name, value)
    degs = floatToString(8, value)
    rads = floatToString(8, ((value*math.pi)/180))
	subtree:add(data, name .. ": " .. rads .. " rad (" .. degs .. " deg)")
end

function displayKelvinTemperature(data, subtree, name, value)
    kelvins = floatToString(8, value)
    degrees = floatToString(8, (value - 273.15) * 9/5 + 32)
    subtree:add(data, name .. ": " ..  kelvins .. " K (" .. degrees .. " F)")
end
function displayRadiansPer(data, subtree, name, value, timeUnit)
    rads = floatToString(8, value)
    degs = floatToString(8, ((value*180)/math.pi))
	subtree:add(data, name .. ": " .. rads .. " rad/" .. timeUnit .. " (" .. degs .." deg/" .. timeUnit .. ")")
end

function displayAsciiString(length, subtree, name, data, index, size, scaler)
    if length > size - index then
        length = size - index
    end
    subtree:add(data(index,length), name .. ": \""  .. data(index,length):string(ENC_ASCII) .. "\"")
    return index + length
end

function displayElevationMask(subtree, name, data, index, size, scaler)
    local elevation = data(index,size):bitfield(4,4) + 0
    subtree:add(data(index,size), name .. ": "  .. floatToString(6, elevation * (1.0/171.887339)))
    return index + size
end

function displayEuler(subtree, name, data, index, size, scaler)
    local eulerArray = subtree:add(data(index,size), name)    
    displayRadians(data(index, 2), eulerArray, "Roll", data(index,2):int() * (1.0/10430.06))
    displayRadians(data(index+2, 2), eulerArray, "Pitch", data(index+2,2):int() * (1.0/10430.06))
    displayRadians(data(index+4, 2), eulerArray, "Yaw", data(index+4,2):int() * (1.0/10430.06))
    return index + size
end

function displayMetersOffset(subtree, name, data, index, size, scaler)
    local metersOffset = subtree:add(data(index,size), name)
    metersOffset:add(data(index, 2),  "X: " .. data(index,2):int() * 0.001 .. " m")
    metersOffset:add(data(index+2, 2), "Y: " .. data(index+2,2):int() * 0.001 .. " m")
    metersOffset:add(data(index+4, 2), "Z: " .. data(index+4,2):int() * 0.001 .. " m")
    return index + size
end

function displayQuat(subtree, name, data, index, size, scaler)
    local quatArray = subtree:add(data(index,size), name)

    W = data(index,2):int() / 32768.0
	X = data(index + 2,2):int() / 32768.0
	Y = data(index + 4,2):int() / 32768.0
	Z = data(index + 6,2):int() / 32768.0

    -- Display Quat data 
	quatArray:add(data(index,2), "W: " .. W)
	quatArray:add(data(index+2,2), "X: " .. X)
	quatArray:add(data(index+4,2), "Y: " .. Y)
    quatArray:add(data(index+6,2), "Z: " .. Z)

    -- Display Euler
	euler = quat_to_euler(W, X, Y, Z)
	displayRadians(data(index,size), quatArray, "Roll", euler[0])
	displayRadians(data(index,size), quatArray, "Pitch", euler[1])
	displayRadians(data(index,size), quatArray, "Yaw", euler[2])
    return index + size
end

function displayDirRMS(subtree, name, data, index, size, scaler)
    subtree:add(data(index, 2),  "Pan Direct Current Jitter: " .. data(index,2):uint() / scaler .. " uA")
    subtree:add(data(index + 2, 2),  "Tilt Direct Current Jitter: " .. data(index + 2, 2):uint() / scaler .. " uA")
    return index + size
end

function displayRMSJitter(subtree, name, data, index, size, scaler)
	subtree:add(data(index, 2),  "Pan Quadrature Current Jitter: " .. data(index, 2):uint() / scaler .. " uA")
	subtree:add(data(index + 2,2),  "Tilt Quadrature Current Jitter: " .. data(index + 2, 2):uint() / scaler .. " uA")
    return index + size
end

function displayVelRMS(subtree, name, data, index, size, scaler)
    subtree:add(data(index, 2),  "Pan Velocity Jitter: " .. data(index, 2):uint() / scaler .. " mrad/s")
    subtree:add(data(index + 2, 2),  "Tilt Velocity Jitter: " .. data(index + 2, 2):uint() / scaler .. " mrad/s")
    return index + size
end

function displayPosRMS(subtree, name, data, index, size, scaler)
    subtree:add(data(index, 2), "Pan Position Jitter: " .. data(index, 2):uint() / scaler .. " radians")
    subtree:add(data(index + 2, 2), "Tilt Position Jitter: " .. data(index + 2, 2):uint() / scaler .. " radians")
    return index + size
end

function displayOutputCurrent(subtree, name, data, index, size, scaler)
    subtree:add(data(index, 2), "Pan Output Current: " .. data(index, 2):uint() / scaler .. " Amps")
    subtree:add(data(index + 2, 2), "Tilt Output Current: " .. data(index + 2, 2):uint() / scaler .. " Amps")
    return index + size
end

function displayLoad(subtree, name, data, index, size, scaler)
    subtree:add(data(index,size), name .. ": " .. floatToString(2, (data(index,size):uint() / scaler) *  (1.0/ 2.55)) .. " %")
    return index + size
end

function displayMaxShift(subtree, name, data, index, size, scaler)
    subtree:add(data(index,size), name .. ": " .. floatToString(6, (data(index,size):uint() / scaler) / 255))
    return index + size
end

function displayIterations(subtree, name, data, index, size, scaler)
    subtree:add(data(index,size), name .. ": " .. floatToString(2, (data(index,size):uint() * (1.0/ scaler))))
    return index + size
end

function displayWatts(subtree, name, data, index, size, scaler)
    local panTilt = subtree:add(data(index,size), name) 
    panTilt:add(data(index,1),  "Pan: "  .. floatToString(4,data(index,1):uint() / scaler) .. " watts")
    panTilt:add(data(index+1,1), "Tilt: "  .. floatToString(4,data(index+1,1):uint() / scaler) .. " watts")
    return index + size
end

function displayWorstCase(subtree, name, data, index, size, scaler)
    subtree:add(data(index,size), name .. ": " .. floatToString(2, (data(index,size):uint() / scaler) / 10))
    return index + size
end

function displayOutputShifts(subtree, name, data, index, size, scaler)
    pantilt = subtree:add(OrionPublic, data(index,size), name)
    pan = (data(index,2):int() / scaler) / 10430.06
    tilt = (data(index+2, 2):int() / scaler) / 10430.06
    displayRadians(data(index, 2), pantilt, "Pan", pan)
    displayRadians(data(index+2, 2), pantilt, "Tilt", tilt)
    return index + size
end

----------------------------- End Display Methods --------------------------------------------

local function compute_max_sig(sigbits)
return (2 ^ sigbits) - 1
end

local function compute_bias(width, sigbits)
return (2 ^ (((width - 2) - sigbits))) - 1
end

local BINARY32_SIG_BITS = 23
local BINARY32_SIG_MASK = 0x007FFFFF
local BINARY32_LEADING1 = 0x00800000
local BINARY32_BIAS = 127

-- value: unsigned 16-bit integer, sigbits: number of significand bits in the source format
local function float16ToFloat32(value, sigbits)
  local fieldInteger = 0

  -- Zero (signed zero preserved later)
  if (value & 0x7FFF) == 0 then
    fieldInteger = 0
  else
    local sigmask = compute_max_sig(sigbits)
    local bias = compute_bias(16, sigbits)

    local unsignedExponent = ((value & 0x7FFF) >> sigbits)

    if unsignedExponent == 0 then
      -- denormalized
      fieldInteger = (value & sigmask) << ((BINARY32_SIG_BITS + 1) - sigbits)
      unsignedExponent = unsignedExponent + (BINARY32_BIAS - bias)

      -- normalize
      while (fieldInteger & 0x00800000) == 0 do
        fieldInteger = fieldInteger << 1
        unsignedExponent = unsignedExponent - 1
      end

      -- mask off the implicit leading 1
      fieldInteger = fieldInteger & BINARY32_SIG_MASK
    else
      fieldInteger = (value & sigmask) << (BINARY32_SIG_BITS - sigbits)
      unsignedExponent = unsignedExponent + (BINARY32_BIAS - bias)
    end

    fieldInteger = fieldInteger | (unsignedExponent << BINARY32_SIG_BITS)
  end

  if (value & 0x8000) ~= 0 then
    fieldInteger = fieldInteger | 0x80000000
  end

  -- reinterpret 32-bit integer bits as IEEE-754 float
  local packed = string.pack(">I4", fieldInteger)
  local f = string.unpack(">f", packed)
  return f
end

-------- End Helper functions ------------------------------------------------------------------------
-- Enumeration Functions

function getTrackMode(subtree, name, data, index, size, scaler)
    local highNoiseCompensation = data(index,size):bitfield(0, 1)
    local acquisitionAssist = data(index,size):bitfield(1, 1)
    local intelligentAssist = data(index,size):bitfield(2, 1)
    local reducedSearch = data(index,size):bitfield(3, 1)
    local trackModeValue = data(index,size):bitfield(4, 4)
    local trackModeString = ""
    if trackModeValue == 0 then
        trackModeString = "No Change"
    elseif trackModeValue == 1 then
        trackModeString = "Stationary Mode"
    elseif trackModeValue == 2 then
        trackModeString = "Vehicle Mode"
    elseif trackModeValue == 3 then
        trackModeString = "Person Mode"
    elseif trackModeValue == 4 then
        trackModeString = "Scene Mode"
    elseif trackModeValue == 6 then
        trackModeString = "Static Mode"
    elseif trackModeValue == 7 then
        trackModeString = "No Registration"
    elseif trackModeValue == 8 then
        trackModeString = "Drone Mode"
    else
        trackModeString = "Reserved"
    end
    trackModeTree = subtree:add(OrionPublic, data(index,size), name)
    trackModeTree:add(data(index,size):bitfield(4, 4), name .. ": " .. trackModeString)
    if highNoiseCompensation == 1 then
        trackModeTree:add(data(index,size):bitfield(0, 1), "High Noise Compensation: Enabled")
    else
        trackModeTree:add(data(index,size):bitfield(0, 1), "High Noise Compensation: Disabled")
    end
    if acquisitionAssist == 1 then
        trackModeTree:add(data(index,size):bitfield(1, 1), "Acquisition Assist: Enabled")
    else
        trackModeTree:add(data(index,size):bitfield(1, 1), "Acquisition Assist: Disabled")
    end
    if intelligentAssist == 1 then
        trackModeTree:add(data(index,size):bitfield(2, 1), "Intelligent Assist: Enabled")
    else
        trackModeTree:add(data(index,size):bitfield(2, 1), "Intelligent Assist: Disabled")
    end
    if reducedSearch == 1 then
        trackModeTree:add(data(index,size):bitfield(3, 1), "Reduced Search: Enabled")
    else
        trackModeTree:add(data(index,size):bitfield(3, 1), "Reduced Search: Disabled")
    end
    return index + size
end

function getDetectionMode(subtree, name, data, index, size, scaler)
    local disableAll = data(index,size):bitfield(15, 1)
    detectionModeTree = subtree:add(OrionPublic, data(index,size), name)
    if disableAll == 1 then
        detectionModeTree:add(data(index,size):bitfield(15, 1), name .. ": All Detection Disabled")
    else         
        detectionModeTree:add(data(index,size):bitfield(15, 1), name .. ": Detection Enabled")
        local resetDetect = data(index,size):bitfield(14, 1)
        local vehicleEnabled = data(index,size):bitfield(11, 1)
        local staringEnabled = data(index,size):bitfield(10, 1)
        local aerialEnabled = data(index,size):bitfield(9, 1)
        local anomalyEnabled = data(index,size):bitfield(8, 1)
        local maritimeEnabled = data(index,size):bitfield(7, 1)
        local radiometricEnabled = data(index,size):bitfield(6, 1)
        local blobEnabled = data(index,size):bitfield(5, 1)
        local droneEnabled = data(index,size):bitfield(4, 1)
        local gasEnabled = data(index,size):bitfield(3, 1)
        local personEnabled = data(index,size):bitfield(1, 1)
        if resetDetect == 1 then
            detectionModeTree:add(data(index,size):bitfield(14, 1), "Reset Detect: Enabled")
        else
            detectionModeTree:add(data(index,size):bitfield(14, 1), "Reset Detect: Disabled")
        end
        if vehicleEnabled == 1 then
            detectionModeTree:add(data(index,size):bitfield(11, 1), "Vehicle Detection: Enabled")
        else
            detectionModeTree:add(data(index,size):bitfield(11, 1), "Vehicle Detection: Disabled")
        end
        if staringEnabled == 1 then
            detectionModeTree:add(data(index,size):bitfield(10, 1), "Staring Detection: Enabled")
        else
            detectionModeTree:add(data(index,size):bitfield(10, 1), "Staring Detection: Disabled")
        end
        if aerialEnabled == 1 then
            detectionModeTree:add(data(index,size):bitfield(9, 1), "Aerial Detection: Enabled")
        else
            detectionModeTree:add(data(index,size):bitfield(9, 1), "Aerial Detection: Disabled")
        end
        if anomalyEnabled == 1 then
            detectionModeTree:add(data(index,size):bitfield(8, 1), "Anomaly Detection: Enabled")
        else
            detectionModeTree:add(data(index,size):bitfield(8, 1), "Anomaly Detection: Disabled")
        end
        if maritimeEnabled == 1 then
            detectionModeTree:add(data(index,size):bitfield(7, 1), "Maritime Detection: Enabled")
        else
            detectionModeTree:add(data(index,size):bitfield(7, 1), "Maritime Detection: Disabled")
        end
        if radiometricEnabled == 1 then
            detectionModeTree:add(data(index,size):bitfield(6, 1), "Radiometric Detection: Enabled")
        else
            detectionModeTree:add(data(index,size):bitfield(6, 1), "Radiometric Detection: Disabled")
        end
        if blobEnabled == 1 then
            detectionModeTree:add(data(index,size):bitfield(5, 1), "Blob Detection: Enabled")
        else
            detectionModeTree:add(data(index,size):bitfield(5, 1), "Blob Detection: Disabled")
        end
        if droneEnabled == 1 then
            detectionModeTree:add(data(index,size):bitfield(4, 1), "Drone Detection: Enabled")
        else
            detectionModeTree:add(data(index,size):bitfield(4, 1), "Drone Detection: Disabled")
        end
        if gasEnabled == 1 then
            detectionModeTree:add(data(index,size):bitfield(3, 1), "Gas Detection: Enabled")
        else
            detectionModeTree:add(data(index,size):bitfield(3, 1), "Gas Detection: Disabled")
        end
        if personEnabled == 1 then
            detectionModeTree:add(data(index,size):bitfield(1, 1), "Person Detection: Enabled")
        else
            detectionModeTree:add(data(index,size):bitfield(1, 1), "Person Detection: Disabled")
        end
    end

    return index + size
end

-- Conversion Functions
function signed16ToGeoidUndulationMeters(subtree, name, data, index, size, scaler)
	subtree:add(data(index,size), name .. ": " .. floatToString(6, data(index,size):int() / 32768.0 * 120.0) .. " m")
    return index + size
end

function uint8ToFloat(subtree, name, data, index, size, scaler)
	subtree:add(data(index,size), name .. ": " .. floatToString(10, (data(index,size):uint() / scaler) / 255.0)) 
    return index + size
end

-- Pan And Til Conversions - these are commonly used in the ICD and have a standard conversion, so they get their own functions. The scaler is used to convert from the raw integer value to radians.
function signed16x2ToPanTilt(subtree, name, data, index, size, scaler)
    pantilt = subtree:add(OrionPublic, data(index,size), name)
    pan = data(index,2):int() / scaler
    tilt = data(index+2, 2):int() / scaler
    displayRadians(data(index, 2), pantilt, "Pan", pan)
    displayRadians(data(index+2, 2), pantilt, "Tilt", tilt)
    return index + 4
end

function unsigned16x2ToPanTiltAccel(subtree, name, data, index, size, scaler)
    pantilt = subtree:add(OrionPublic, data(index,size), name)
    pan = data(index,2):uint() / scaler
    tilt = data(index+2, 2):uint() / scaler
    displayRadiansPer(data(index, 4), pantilt, "Pan", pan, "s/s")
    displayRadiansPer(data(index+4, 4), pantilt, "Tilt", tilt, "s/s")
    return index + 4
end

function unsigned16x2ToPanTiltVel(subtree, name, data, index, size, scaler)
    pantilt = subtree:add(OrionPublic, data(index,size), name)
    pan = data(index,2):uint() / scaler
    tilt = data(index+2, 2):uint() / scaler
    displayRadiansPer(data(index, 4), pantilt, "Pan", pan, "s")
    displayRadiansPer(data(index+4, 4), pantilt, "Tilt", tilt, "s")
    return index + 4
end
function signed32x2ToPanTilt(subtree, name, data, index, size, scaler)
    pantilt = subtree:add(OrionPublic, data(index,size), name)
    pan = data(index,4):int() / scaler
    tilt = data(index+4, 4):int() / scaler
    displayRadians(data(index, 4), pantilt, "Pan", pan)
    displayRadians(data(index+4, 4), pantilt, "Tilt", tilt)
    return index + 8
end

function signed16ToPanTilt(subtree, name, data, index, size, scaler)
    displayRadians(data(index, size), subtree, name, data(index, size):int() * 1.0/10430.06)
    return index + size
end

function signed16x2ToPanTiltOffset(subtree, name, data, index, size, scaler)
    pantiltoffset = subtree:add(OrionPublic, data(index,4), name .. " Offsets")
    displayRadians(data(index,2), pantiltoffset, "Pan Offset", data(index,2):int() * 1.0/10430.06)
    displayRadians(data(index+2,2), pantiltoffset, "Tilt Offset", data(index+2,2):int() * 1.0/10430.06)
    return index + 4
end
--- End Pan and Tilt Conversions

-- Start General Conversions - 
function unsigned8ToEnabled(subtree, name, data, index, size, scaler)
    local enabled = data(index,size):uint()
    if enabled == 0 then
        subtree:add(data(index,size), name .. ": Disabled")
    else
        subtree:add(data(index,size), name .. ": Enabled")
    end

    return index + size
end

function unsigned8ToDisabled(subtree, name, data, index, size, scaler)
    local disabled = data(index,size):uint()
    if disabled == 1 then
        subtree:add(data(index,size), name .. ": Disabled")
    else
        subtree:add(data(index,size), name .. ": Enabled")
    end

    return index + size
end

function bitfield1ToEnabled(subtree, name, data, index, size, scaler, bitIndex)
    local enabled = data(index,size):bitfield(bitIndex,1)
    if enabled == 0 then
        subtree:add(data(index,size), name .. ": Disabled")
    else
        subtree:add(data(index,size), name .. ": Enabled")
    end

    return index -- bitfield1, do not increment index
end

function bitfield1ToYes(subtree, name, data, index, size, scaler, bitIndex)
    local enabled = data(index,size):bitfield(bitIndex,1)
    if enabled == 0 then
        subtree:add(data(index,size), name .. ": No")
    else
        subtree:add(data(index,size), name .. ": Yes")
    end
    return index -- bitfield1, do not increment index
end

function bitfield1ToTrue(subtree, name, data, index, size, scaler, bitIndex)
    local enabled = data(index,size):bitfield(bitIndex,1)
    if enabled == 0 then
        subtree:add(data(index,size), name .. ": False")
    else
        subtree:add(data(index,size), name .. ": True")
    end
    return index -- bitfield1, do not increment index
end


function unsigned8ToFloat32(subtree, name, data, index, size, scaler)
    subtree:add(data(index,size), name .. ": " .. floatToString(4, (data(index,size):uint() / scaler) * (1.0/255.0)))
    return index + size
end

function displayAsRadians(subtree, name, data, index, size, scaler)
    displayRadians(data(index,size), subtree, name, data(index,size):float() / scaler)
    return index + size
end

-- End Track Conversions

--- Measurement Conversions - these are used to convert raw integer values into human readable measurements.
function unsigned32ToUMScaled(subtree, name, data, index, size, scaler)
    subtree:add(data(index,size), name .. ": " .. floatToString(2, (data(index,size):uint() / scaler)) .. " mm (" .. data(index,size):uint() .. " µm)")
    return index + size
end

function unsigned16ToNMPixelsScaled(subtree, name, data, index, size, scaler)
    subtree:add(data(index,size), name .. ": " .. floatToString(8, (data(index,size):uint() / scaler)) .. " pm (" .. data(index,size):uint() .. " nm)")
    return index + size
end
--- End Measurement Conversions

function unsigned32ToBytes(subtree, name, data, index, size, scaler)
    local unitType = "Bytes"
    local byteCount = data(index,size):uint()
    if byteCount > 1073741824 then
        unitType = "GB"
        byteCount = byteCount / 1073741824
    elseif byteCount > 1048576 then
        unitType = "MB"
        byteCount = byteCount / 1048576
    elseif byteCount > 1024 then
        unitType = "KB"
        byteCount = byteCount / 1024
    end
    subtree:add(data(index,size), name .. ": " .. byteCount .. " " .. unitType)
    return index + size
end

function unsigned32ToIPv4(subtree, name, data, index, size, scaler)
    subtree:add(data(index,size),  name .. ": " .. data(index,1):uint() .. "." .. data(index + 1,1):uint() .. "." .. data(index + 2,1):uint() .. "." .. data(index + 3,1):uint())
    return index + size
end

function unsigned8x4ToIPv4(subtree, name, data, index, size, scaler)
	return unsigned32ToIPv4(subtree, name, data, index, size, scaler)
end

-- Power Conversions - these are used to convert raw integer values into human readable power measurements.
function unsigned16ToVarVolts(subtree, name, data, index, size, scaler)
	subtree:add(data(index,size), name .. ": " .. floatToString(4, data(index,size):uint() / 100) .. " mV")
    return index + size
end

function unsigned16x2ToCurrent(subtree, name, data, index, size, scaler)
    local panTilt = subtree:add(data(index,size), name) 
    panTilt:add(data(index, 2), "Pan: " .. floatToString(3, data(index, 2):uint() / scaler) .. " Amps")
    panTilt:add(data(index + 2, 2), "Tilt: " .. floatToString(3, data(index + 2, 2):uint() / scaler) .. " Amps")
    return index + size
end

function unsigned16ToVarAmps(subtree, name, data, index, size, scaler)
	subtree:add(data(index,size), name .. ": "  .. floatToString(4,data(index,size):uint() / 100) .. " mA")
    return index + size
end

-- End Power Conversions

function unsigned8ToPercent(subtree, name, data, index, size, scaler)
	subtree:add(data(index,size), name .. ": "  .. floatToString(4,data(index,size):uint() / 2.55) .. " %")
    return index + size
end

function unsigned8ToHitachiSharpness(subtree, name, data, index, size, scaler)
    subtree:add(data(index,size), name .. ": "  .. floatToString(4, (data(index,size):uint() / 50.0)))
    return index + size
end

function unsigned8ToGamma(subtree, name, data, index, size, scaler)
    subtree:add(data(index,size), name .. ": "  .. floatToString(6, (data(index,size):uint() / 72.8571429) + 0.5) )
    return index + size
end

function unsigned8ToTemperature(subtree, name, data, index, size, scaler)
    subtree:add(data(index,size), name .. ": "  .. floatToString(4, data(index,size):uint() / scaler) .. " C" )
    return index + size
end

function exposureToMilliseconds(subtree, name, data, index, size, scaler)
    subtree:add(data(index,size), name .. ": "  .. floatToString(6, (data(index,size):uint() / 1988.92261) + 0.05) .. " ms" )
    return index + size
end

function unsigned8ToStareTime(subtree, name, data, index, size, scaler)
    subtree:add(data(index,size), name .. ": "  .. floatToString(2, (data(index,size):uint() / 100.0)) .. " seconds" )
    return index + size
end

function unsigned8ToStareLoad(subtree, name, data, index, size, scaler)
    subtree:add(data(index,size), name .. ": "  .. floatToString(5, (data(index,size):uint() * (1.0/255) )))
    return index + size
end

function unsigned16ToMinMaxGain(subtree, name, data, index, size, scaler)
    subtree:add(data(index,size), name .. ": "  .. floatToString(5, (data(index,size):uint() / 2114.03226) + 1.0))
    return index + size
end

function cameraContrast(subtree, name, data, index, size, scaler)
    subtree:add(data(index,size), name .. ": "  .. floatToString(6, (data(index,size):int() / 127.0)))
    return index + size
end

function cameraSharpness(subtree, name, data, index, size, scaler)
    subtree:add(data(index,size), name .. ": "  .. floatToString(6, (data(index,size):uint() / 51.0) ))
    return index + size
end

function cameraAperature(subtree, name, data, index, size, scaler)
    subtree:add(data(index,size), name .. ": "  .. floatToString(6, (data(index,size):uint() * (1.0/ 7.96875)) ))
    return index + size
end

function cameraFramerate(subtree, name, data, index, size, scaler)
    subtree:add(data(index,size), name .. ": "  .. floatToString(6, (data(index,size):uint() / 8.5) ))
    return index + size
end

function float16ToChiSquare(subtree, name, data, index, size, scaler)
	subtree:add(data(index,size), name .. ": "  .. floatToString(8, float16ToFloat32(data(index,size):uint(),  9) / scaler))
    return index + size
end

function signed32ToLatRads(subtree, name, data, index, size, scaler)
    displayRadians(data(index, size), subtree, name, (data(index, size):int() / scaler) / 1367130550.516243)
    return index + size
end

function signed32ToLonRads(subtree, name, data, index, size, scaler)
    displayRadians(data(index, size), subtree, name, (data(index, size):int() / scaler) / 683565275.2581217)
    return index + size
end

function unsigned16x3To16QUAT(subtree, name, data, index, size, scaler)
    local quat = subtree:add(data(index,size), name)
    displayRadians(data(index, 2), quat, "Roll", data(index, 2):uint() / scaler)
    displayRadians(data(index + 2, 2),quat, "Pitch", data(index + 2, 2):uint() / scaler)
    displayRadians(data(index + 4, 2),quat, "Yaw", data(index + 4, 2):uint() / scaler)
    return index + size
end

function signed16x4ToQuat(subtree, name, data, index, size, scaler)
    local quat = subtree:add(data(index,size), name)
    displayRadians(data(index, 2), quat, "W", (data(index, 2):int() / scaler) / 32767.0)
    displayRadians(data(index + 2, 2),quat, "X", (data(index + 2, 2):int() / scaler) / 32767.0)
    displayRadians(data(index + 4, 2),quat, "Y", (data(index + 4, 2):int() / scaler) / 32767.0)
    displayRadians(data(index + 6, 2),quat, "Z", (data(index + 6, 2):int() / scaler) / 32767.0)
    return index + size
end

function signed16x3To16QUAT(subtree, name, data, index, size, scaler)
    local quat = subtree:add(data(index,size), name)
    displayRadiansPer(data(index, 2), quat, "Roll", data(index, 2):int() / scaler, "s")
    displayRadiansPer(data(index + 2, 2),quat, "Pitch", data(index + 2, 2):int() / scaler, "s")
    displayRadiansPer(data(index + 4, 2),quat, "Yaw", data(index + 4, 2):int() / scaler, "s")
    return index + size
end

function unsigned16x3ToGyroBias(subtree, name, data, index, size, scaler)
    local quat = subtree:add(data(index,size), name)
    displayRadiansPer(data(index, 2), quat, "Roll", data(index, 2):uint() / scaler, "s")
    displayRadiansPer(data(index + 2, 2),quat, "Pitch", data(index + 2, 2):uint() / scaler, "s")
    displayRadiansPer(data(index + 4, 2),quat, "Yaw", data(index + 4, 2):uint() / scaler, "s")
    return index + size
end

function signed16x3ToAcceleration(subtree, name, data, index, size, scaler)
    local quat = subtree:add(data(index,size), name)
    quat:add(data(index, 2), "Roll: " .. data(index, 2):int() / scaler .. " m/s/s")
    quat:add(data(index + 2, 2), "Pitch: " .. data(index + 2, 2):int() / scaler .. " m/s/s")
    quat:add(data(index + 4, 2), "Yaw: " .. data(index + 4, 2):int() / scaler .. " m/s/s")
    return index + size
end

function float32x3ToAccelerometerBias(subtree, name, data, index, size, scaler)
    local quat = subtree:add(data(index,size), name)
    quat:add(data(index, 2), "X: " .. data(index, 2):int() / scaler .. " m/s/s")
    quat:add(data(index + 2, 2), "Y: " .. data(index + 2, 2):int() / scaler .. " m/s/s")
    quat:add(data(index + 4, 2), "Z: " .. data(index + 4, 2):int() / scaler .. " m/s/s")
    return index + size
end

function unsigned16x3ToSigmaNED(subtree, name, data, index, size, scaler)
    local ned = subtree:add(data(index,size), name)
    ned:add(data(index, 2), "North: " .. floatToString(1, ((data(index, 2):uint() / scaler) / 65.535)) .. " m/s")
    ned:add(data(index+2, 2), "East: " .. floatToString(1, ((data(index+2, 2):uint() / scaler) / 65.535)) .. " m/s")
    ned:add(data(index+4, 2), "Down: " .. floatToString(1, ((data(index+4, 2):uint() / scaler) / 65.535)) .. " m/s")
    return index + size
end

function signed16ToSigmaLat(subtree, name, data, index, size, scaler)
    displayRadians(data(index, size), subtree, name, (data(index, size):int() / scaler) / 20860.12008116854)
    return index + size
end

function signed16ToSigmaLonAlt(subtree, name, data, index, size, scaler)
    displayRadians(data(index, size), subtree, name, (data(index, size):int() / scaler) / 10430.06004058427)
    return index + size
end

function signed16ToCourse(subtree, name, data, index, size, scaler)
    displayDegrees(data(index, size), subtree, name, (data(index, size):int() / scaler) / 91.01944444444445)
    return index + size
end

function unsigned16x3To16NED(subtree, name, data, index, size, scaler)
    local ned = subtree:add(data(index,size), name)
	ned:add(data(index, 2), "North: " .. (data(index, 2):uint() / scaler) .. " m/s")
	ned:add(data(index+2, 2), "East: " .. (data(index+2, 2):uint() / scaler) .. " m/s")
	ned:add(data(index+4, 2), "Down: " .. (data(index+4, 2):uint() / scaler) .. " m/s")
    return index + size
end

function signed16ToEllipseAngle(subtree, name, data, index, size, scaler)
    displayRadians(data(index, size), subtree, name, (data(index, size):int() / scaler) / 10430.06004058427)
    return index + size
end

function signed16ToEllipseValue(subtree, name, data, index, size, scaler)
    subtree:add(data(index,size), name .. ": " .. floatToString(1, data(index,size):int() / scaler / 32.767) .. " meters")
    return index + size
end

function signed16x3To16NED(subtree, name, data, index, size, scaler)
    local ned = subtree:add(data(index,size), name)
	ned:add(data(index, 2), "North: " .. data(index, 2):int() / scaler .. " m/s")
	ned:add(data(index+2, 2), "East: " .. data(index+2, 2):int() / scaler .. " m/s")
	ned:add(data(index+4, 2), "Down: " .. data(index+2, 2):int() / scaler .. " m/s")
    return index + size
end

function signed32x3ToNED(subtree, name, data, index, size, scaler)
    local ned = subtree:add(data(index,size), name)
	ned:add(data(index, 4), "North: " .. (data(index, 4):int() / scaler) .. " m/s")
	ned:add(data(index+4, 4), "East: " .. (data(index+4, 4):int() / scaler) .. " m/s")
	ned:add(data(index+4, 4), "Down: " .. (data(index+8, 4):int() / scaler) .. " m/s")
    return index + size
end

function signed32ToMeterAccuracy(subtree, name, data, index, size, scaler)
	subtree:add(data(index, size), name .. ": " .. data(index, size):int() / scaler .. " m")
    return index + size
end 

function signed32ToSpeed(subtree, name, data, index, size, scaler)
	subtree:add(data(index, size), name .. ": " .. data(index, size):int() / scaler .. " m/s")
    return index + size
end 

function signed8ToDegrees(subtree, name, data, index, size, scaler)
    displayRadians(data(index, size), subtree, name, data(index, size):uint() / scaler)
    return index + size
end

function signed16ToDegrees(subtree, name, data, index, size, scaler)
    displayRadians(data(index, size), subtree, name, data(index, size):uint() / scaler)
    return index + size
end

function signed16ToDegrees(subtree, name, data, index, size, scaler)
    displayRadians(data(index, size), subtree, name, data(index, size):int() / 10430.06)
    return index + size
end

function unsigned16ToDegrees(subtree, name, data, index, size, scaler)
    displayRadians(data(index, size), subtree, name, data(index, size):int() / 10430.2192)
    return index + size
end

function signed32ToDegrees(subtree, name, data, index, size, scaler)
	displayRadians(data(index, size), subtree, name, data(index, size):int() / scaler)
    return index + size
end 


function float32ToDegrees(subtree, name, data, index, size, scaler)
	displayRadians(data(index, size), subtree, name, data(index, size):float() / scaler)
    return index + size
end 

-- Rate Conversions
function unsigned16ToHertz(subtree, name, data, index, size, scaler)
	subtree:add(data(index, size), name .. ": " .. floatToString(4, (data(index, size):uint() / scaler * 1.0/327.675)) .. " hz")
    return index + size
end

function unsigned8x3ToMagnitude(subtree, name, data, index, size, scaler)
    local magnitude = subtree:add(data(index,size), name)
    magnitude:add(data(index, 1), "Roll : " .. floatToString(7, (data(index, 1):uint() / 255.0) / scaler) .. " %")
    magnitude:add(data(index + 1, 1), "Pitch : " .. floatToString(7, (data(index + 1, 1):uint()/ 255.0) / scaler) .. " %")
    magnitude:add(data(index + 2, 1), "Yaw : " .. floatToString(7, (data(index + 2, 1):uint() / 255.0) / scaler) .. " %")
    return index + size
end

-- End Time Conversions


-- Time Conversions - these are used to convert raw integer values into human readable time measurements.
function unsigned16ToMilliseconds(subtree, name, data, index, size, scaler)
	subtree:add(data(index, size), name .. ": " .. (data(index, size):uint() / scaler * 1.0/10430.06) .. " ms")
    return index + size
end

function unsigned8ToIntegrationTime(subtree, name, data, index, size, scaler)
	subtree:add(data(index, size), name .. ": " .. floatToString(6, (data(index, size):uint() / 8.79310345) + 1.0) .. " ms")
    return index + size
end

-- End Time Conversions

function unsigned16ToFOV(subtree, name, data, index, size, scaler)
    displayRadians(data(index, size), subtree, name, data(index, size):int() / 1.0/10430.2192)
    return index + size
end

function convertSigned24FloatToSigned32Float(value, scaler)
    value = (value / 1.0) / scaler
    if(value >= 0) then
        if(value >= 8388607) then
            return 8388607
        else
            if(value == (value // 1)) then
                return value
            else
                return value + 0.5
            end
        end
    else
        if(value <= -8388608) then
            return -8388608
        else
            if(value == (value // 1)) then
                return value
            else
                return value - 0.5
            end
        end
    end
end
function signed24x3ToPoints(subtree, name, data, index, size, scaler)
    subtree:add(data(index, size), name .. "[0]: " .. convertSigned24FloatToSigned32Float(data(index, 3):int(), scaler))
    subtree:add(data(index, size), name .. "[1]: " .. convertSigned24FloatToSigned32Float(data(index+3, 3):int(), scaler))
    subtree:add(data(index, size), name .. "[2]: " .. convertSigned24FloatToSigned32Float(data(index+6, 3):int(), scaler))
    return index + size
end

function unsigned24ToSlantRange(subtree, name, data, index, size, scaler)
	subtree:add(data(index, size), name .. ": " .. data(index, size):int() / scaler .. " m")
    return index + size
end

function signed32ToAltitude(subtree, name, data, index, size, scaler)
	subtree:add(data(index, size), name .. ": " .. data(index, size):int() / 10000.0 .. " m")
    return index + size
end 

function float32To24BitRange(subtree, name, data, index, size, scaler)
	subtree:add(data(index, size), name .. ": " .. data(index, size):bitfield(0,24) / scaler)
    return index + size
end

function unsignedToFloat32(subtree, name, data, index, size, scaler) 
    unsignedValue = data(index, size):uint()
    if(size == 4) then
        unsignedValue = unsignedValue / 1000000.0
    elseif(size == 2) then
        unsignedValue = (unsignedValue / 2114.03226) + 1.0
    elseif (size == 1) then
        unsignedValue = unsignedValue / 255
    end
	subtree:add(data(index, size), name .. ": " .. unsignedValue / scaler)
    return index + size
end

function signedToFloat32(subtree, name, data, index, size, scaler) 
    unsignedValue = data(index, size):uint()
    if(size == 4) then
        unsignedValue = unsignedValue / 1000000.0
    elseif(size == 2) then
        unsignedValue = unsignedValue * (1.0/2114.03226)
    elseif (size == 1) then
        unsignedValue = unsignedValue * (1.0/127)
    end
	subtree:add(data(index, size), name .. ": " .. unsignedValue / scaler)
    return index + size
end

function signed16ToStareAngle(subtree, name, data, index, size, scaler)
	subtree:add(data(index,size), name .. string.format(" Stare Angle: %.1f", data(index,size):int() / 32768.0 * 180.0))
    return index + size
end

function signed16x3ToAngularRate(subtree, name, data, index, size, scaler)
    local anglerate = subtree:add(data(index,size), name)
    displayRadiansPer(data(index,2), anglerate, "Roll", (data(index,2):int() / 6258.03602) / scaler,  "s")
    displayRadiansPer(data(index+2,2), anglerate, "Pitch", (data(index+2,2):int() / 6258.03602) / scaler,  "s")
    displayRadiansPer(data(index+4,2), anglerate, "Yaw", (data(index+4,2):int() / 6258.03602) / scaler,  "s")
    return index + size
end


function signed16x3ToAxesForce(subtree, name, data, index, size, scaler)
    local axesForce = subtree:add(data(index,size), name)
    axesForce:add(data(index,2), "Roll: " .. floatToString(6, data(index,2):int() * (1.0/334.01631) / scaler) .. " m/s")
    axesForce:add(data(index+2,2), "Pitch: " .. floatToString(6, data(index+2,2):int() * (1.0/334.01631) / scaler) .. " m/s")
    axesForce:add(data(index+4,2), "Yaw: " .. floatToString(6, data(index+4,2):int() * (1.0/334.01631) / scaler) .. " m/s")
    return index + size
end

function unsigned16ToBarometricPressure(subtree, name, data, index, size, scaler)
    subtree:add(data(index,size), name .. ": " .. floatToString(4, ((data(index,size):uint() /0.595772727) / scaler)) .. " Pa")
    return index + size
end

function unsigned16ToKelvinTemperature(subtree, name, data, index, size, scaler)
    displayKelvinTemperature(data(index,size), subtree, name, data(index,size):uint() / scaler)
    return index + size
end

function unsigned8ToDetectState(subtree, name, data, index, size, scaler)
    local state = data(index,size):uint()
    local stateString = "Invalid"
    if state == 0 then
        stateString = "Disable"
    elseif state == 1 then
        stateString = "Enable"
    elseif state == 2 then
        stateString = "Faux"
    end
    subtree:add(data(index,size), name .. ": " .. stateString)
    return index + size
end

function unsigned8ToTrackState(subtree, name, data, index, size, scaler)
    local state = data(index,size):uint()
    local stateString = "Invalid"
    if state == 0 then
        stateString = "Not Tracking"
    elseif state == 1 then
        stateString = "Tracking"
    end
    subtree:add(data(index,size), name .. ": " .. stateString)
    return index + size
end

function unsigned64ToFauxPRFs(subtree, name, data, index, size, scaler)
    local fauxPRFs = subtree:add(data(index,size), name)
    for i = 0, size-1 do -- 8 bytes
        local prfhigh = data(index + i, 1):bitfield(0, 4)
        local prflow = data(index + i, 1):bitfield(4, 4)
        fauxPRFs:add(data(index + i, 1), "Faux PRF[" .. (i * 2) .. "]: " .. prfhigh)
        fauxPRFs:add(data(index + i, 1), "Faux PRF[" .. (i * 2) + 1 .. "]: " .. prflow)
    end
    return index + size
end

-- convert latitude, longitude, altitude to ECEF coordinates. Latitude and Longitude should be in radians, altitude should be in meters. ECEF coordinates are returned in meters.
function lla_to_ecef(lat, lon, alt)
	local sinLat = math.sin(lat)
	local cosLat = math.cos(lat)
	local sinLon = math.sin(lon)
	local cosLon = math.cos(lon)

    -- // Radius of East-West curvature in meters
    local Rc = datum_semiMajorAxis / math.sqrt(1.0 - datum_eSquared * sinLat * sinLat)
    local ecef = {}

    -- // PosECEF position data
    ecef[0] = (Rc+alt)*cosLat*cosLon
    ecef[1] = (Rc+alt)*cosLat*sinLon
    ecef[2] = (Rc*(1.0 - datum_eSquared) + alt)*sinLat

    return ecef
end
-- Convert ECEF coordinates to LLA. Latitude and Longitude are returned in radians, altitude is returned in meters.
function ecef_to_lla(x, y, z)
    local psquared = x*x + y*y
    local cosLat, sinLat, cosLon, sinLon

	local lla = {}

    if psquared == 0.0 then
        -- // We are on the Earth rotation axis, we could be
        -- // in the center, or at one of the poles
        lla[2] = math.abs(z) - datum_semiMinorAxis
        lla[1] = 0.0
        cosLon = 1.0
        sinLon = 0.0

        if z == 0.0 then
            lla[0] = 0.0
        elseif z > 0.0 then
            lla[0] = 3.1415926535859/2.0
        else
            lla[0] = -3.1415926535859/2.0
        end
    else
        -- // distance from axis of rotation
        local p = math.sqrt(psquared)

        local zeta = math.atan(z*datum_semiMajorAxis, p*datum_semiMinorAxis)
        local SinZeta = math.sin(zeta)
        local CosZeta = math.cos(zeta)

        -- // Latitude
        local num = z + datum_eSecondSquared*datum_semiMinorAxis*SinZeta*SinZeta*SinZeta
        local den = p - datum_eSquared*datum_semiMajorAxis*CosZeta*CosZeta*CosZeta

        -- // hypotenuse should never be zero
        local hyp = math.sqrt(num*num + den*den)
        lla[0] = math.atan(num, den)
        sinLat = num/hyp
        cosLat = den/hyp

        -- // Longitude
        lla[1] = math.atan(y, x)

        -- // Altitude is calculated differently at the poles, in order to avoid the singularity
        if math.abs(cosLat) > 0.001 then
            lla[2] = (p / cosLat) - datum_semiMajorAxis / math.sqrt(1.0 - datum_eSquared * sinLat * sinLat)
        else
            lla[2] = math.abs(z) - datum_semiMinorAxis
        end
    end

    return lla

end

function quat_to_euler(a, b, c, d)
	local angles = {}
	angles[0] = math.atan(2*(c*d + b*a), a^2 - b^2 - c^2 + d^2)
	angles[1] = math.asin(math.max(-1, math.min(-2*(b*d - c*a), 1.0)))
	angles[2] = math.atan(2*(b*c + d*a), a^2 + b^2 - c^2 - d^2)
	return angles
end
---------------- End Conversion Methods ------------------------------------------------------------


