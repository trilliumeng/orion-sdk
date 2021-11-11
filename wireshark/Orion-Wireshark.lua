-------------------------------------------------------------------------------
-- Orion protocol dissector for Wireshark
--
-- To use, place in the "User plugins" folder. Orion packets can be filtered
--   as follows:
--
--  * orion
--      This filter shows all frames which contain Orion packets
--  * orion.id == 0x01
--      Use this form to show only frames which contain orion packets with ID
--      0x01 (ORION_PKT_CMD)
--  * orion.length == 7
--      Use this form to show only frames which contain Orion packets with
--      a data payload length of 7
--
-- NOTES:
--  * This is not a complete parser - unimplemented packets will just show the
--      type ID and raw hex dump of the packet data
--  * Checksums are not validated by this module. This means that any frame on
--      TCP port 8747 or UDP port 8748 which *starts* with the Orion packet
--      sync bytes (0xd0 0x0d) will be marked as Orion packets
--
-------------------------------------------------------------------------------

-- Declare the Orion Protocol
orion = Proto("orion","Orion Protocol")

-- the new fields that contain the extracted data (one in string form, one in hex)
orion_id = ProtoField.new("Packet Type ID", "orion.id", ftypes.UINT8)
orion_len = ProtoField.new("Data Length", "orion.length", ftypes.UINT8)
orion_systime = ProtoField.new("System Time", "orion.time", ftypes.UINT32)

-- register the new fields into our fake protocol
orion.fields = { orion_id, orion_len, orion_systime }

function get_ip_string(ipv4)
	local ip_string = ""

	ip_string = ip_string .. ipv4(0,1):uint() .. "."
	ip_string = ip_string .. ipv4(1,1):uint() .. "."
	ip_string = ip_string .. ipv4(2,1):uint() .. "."
	ip_string = ip_string .. ipv4(3,1):uint()

	return ip_string
end

function get_range_src_string(range_src)
	local range_str = "Unknown"

	if range_src == 0 then
		range_str = "None"
	elseif range_src == 1 then
		range_str = "SkyLink"
	elseif range_src == 2 then
		range_str = "Laser"
	elseif range_src == 3 then
		range_str = "Other"
	elseif range_src == 4 then
		range_str = "Internal"
	elseif range_src == 5 then
		range_str = "Internal DTED"
	end

	return range_str
end

function get_mode_string(mode)

	local mode_str = "Unknown Mode 0x" .. mode

	if mode == 0 then mode_str = "Disabled"
	elseif mode == 16 then mode_str = "Rate"
	elseif mode == 32 then mode_str = "FFC Auto"
	elseif mode == 33 then mode_str = "FFC Manual"
	elseif mode == 48 then mode_str = "Scene"
	elseif mode == 49 then mode_str = "Track"
	elseif mode == 64 then mode_str = "Calibration"
	elseif mode == 80 then mode_str = "Position"
	elseif mode == 96 then mode_str = "Geopoint"
	elseif mode == 112 then mode_str = "Path"
	elseif mode == 113 then mode_str = "Look Down"
	end

	return mode_str

end

function get_gps_string(gps_source)

    local gps_string = "Other"
    if gps_source == 0 then gps_string = "External"
	elseif gps_source == 1 then gps_string = "U-Blox"
	elseif gps_source == 2 then gps_string = "Mavlink"
	elseif gps_source == 3 then gps_string = "NMEA"
	elseif gps_source == 4 then gps_string = "Novatel"
	elseif gps_source == 5 then gps_string = "Piccolo"
	end

	return gps_string

end

function get_board_string(board)

	local board_str = "Unknown"

	if board == 1 then board_str = "Clevis"
	elseif board == 2 then board_str = "INS"
	elseif board == 3 then board_str = "Payload"
	elseif board == 4 then board_str = "Lens Control"
	end

	return board_str

end

datum_semiMajorAxis = 6378137.0
datum_flattening = (1.0 / 298.257223563)
datum_semiMinorAxis = (datum_semiMajorAxis * (1.0 - datum_flattening))
datum_eSquared = (1.0 - (datum_semiMinorAxis * datum_semiMinorAxis) / (datum_semiMajorAxis * datum_semiMajorAxis))
datum_eSecondSquared = ((datum_semiMajorAxis * datum_semiMajorAxis) / (datum_semiMinorAxis * datum_semiMinorAxis) - 1.0)

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

        local zeta = math.atan2(z*datum_semiMajorAxis, p*datum_semiMinorAxis)
        local SinZeta = math.sin(zeta)
        local CosZeta = math.cos(zeta)

        -- // Latitude
        local num = z + datum_eSecondSquared*datum_semiMinorAxis*SinZeta*SinZeta*SinZeta
        local den = p - datum_eSquared*datum_semiMajorAxis*CosZeta*CosZeta*CosZeta

        -- // hypotenuse should never be zero
        local hyp = math.sqrt(num*num + den*den)
        lla[0] = math.atan2(num, den)
        sinLat = num/hyp
        cosLat = den/hyp

        -- // Longitude
        lla[1] = math.atan2(y, x)

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

	angles[0] = math.atan2(2*(c*d + b*a), a^2 - b^2 - c^2 + d^2)
	angles[1] = math.asin(math.max(-1, math.min(-2*(b*d - c*a), 1.0)))
	angles[2] = math.atan2(2*(b*c + d*a), a^2 + b^2 - c^2 - d^2)

	return angles
end

function make_subtree(subtree, buffer, name, id, size)
	subtree = subtree:add(orion, buffer, name)
	subtree:add(orion_id, id)
	subtree:add(orion_len, size)
	return subtree
end

function print_cmd(subtree, buffer, id, size)
	local name =  "Command"
	subtree = make_subtree(subtree, buffer, name, id, size)

	if size == 0 then
		return name
	end

	subtree:add(buffer(0,2), "Pan target: " .. math.deg(buffer(0,2):int() / 1000.0))
	subtree:add(buffer(2,2), "Tilt target: " .. math.deg(buffer(2,2):int() / 1000.0))

	local mode = buffer(4,1):uint()

	subtree:add(buffer(4,1), "Mode: " .. get_mode_string(mode))
	subtree:add(buffer(5,1), "Stabilized: " .. buffer(5,1):uint())
	subtree:add(buffer(6,1), "Impulse Time: " .. buffer(6,1):uint() / 10.0)

	return name
end

function print_lasers(subtree, buffer, id, size)
	local name =  "Lasers"
	subtree = make_subtree(subtree, buffer, name, id, size)

	if size == 0 then
		return name
	end

	local count = buffer(0,1):uint()

	local start = 1

	for i = 0,count-1 do
		local p0_len = 1
		local p1_len = 1

		while buffer(start + 5 + p0_len,1):uint() > 0 do
			p0_len = p0_len + 1
		end

		while buffer(start + 5 + p0_len + p1_len,1):uint() > 0 do
			p1_len = p1_len + 1
		end

		local data = buffer(start,15 + p0_len + p1_len)
		laser = subtree:add(data, "Laser " .. i)

		laser:add(data(0,1), "Type: " .. data(0,1):uint())
		laser:add(data(1,2), "Can Enable: " .. data(1,2):bitfield(7,1))
		laser:add(data(1,2), "Disable Shift: " .. data(1,2):bitfield(12,1))
		laser:add(data(1,2), "Fire On Start: " .. data(1,2):bitfield(13,1))
		laser:add(data(1,2), "Eye Safe: " .. data(1,2):bitfield(14,1))
		laser:add(data(1,2), "Can Arm: " .. data(1,2):bitfield(15,1))

        laser:add(data(3,2), "Max Active Time: ".. data(3,2):uint() * 0.001) -- inMemoryType="float32" encodedType="unsigned16" scaler="1000.0" comment="Maximum continuous firing time, in seconds" />
        laser:add(data(5,p0_len), "Password: ".. data(5,p0_len)) -- inMemoryType="string" array="16" comment="Current password, needed to allow overwriting of the current settings" />
        laser:add(data(5 + p0_len, p1_len), "New Password: ".. data(5 + p0_len,p1_len)) -- inMemoryType="string" array="16" comment="Password to use to lock out state changes. Use an empty string to disable password checking" />
        laser:add(data(5 + p0_len + p1_len, 2), "Min Ground Speed: ".. data(5 + p0_len + p1_len, 2):int()) -- inMemoryType="signed16" comment="Minimum GPS ground speed for laser activation, in m/s. Values less than zero are treated as no restriction" />
        laser:add(data(7 + p0_len + p1_len, 2), "Min Altitude: ".. data(7 + p0_len + p1_len, 2):int()) -- inMemoryType="signed16" comment="Minimum GPS altitude for laser activation, in m/s. Values less than zero are treated as no restriction" />
        laser:add(data(9 + p0_len + p1_len, 2), "Pan Alignment: ".. math.deg(data(9 + p0_len + p1_len, 2):int() / 32768.0 * math.pi / 4)) -- inMemoryType="float32" array="NUM_GIMBAL_AXES" encodedType="signed16" max="pi/4" />
        laser:add(data(11 + p0_len + p1_len, 2), "Tilt Alignment: ".. math.deg(data(11 + p0_len + p1_len, 2):int() / 32768.0 * math.pi / 4)) -- inMemoryType="float32" array="NUM_GIMBAL_AXES" encodedType="signed16" max="pi/4" />
        laser:add(data(13 + p0_len + p1_len, 1), "Use AP Data: ".. data(13 + p0_len + p1_len, 1):uint()) -- inMemoryType="unsigned8" comment="Set to 1 to incorporate autopilot data into laser safety checks" />

        start = start + p0_len + p1_len + 15
	end

	return name
end

function print_laser_states(subtree, buffer, id, size)
	local name =  "Laser States"
	subtree = make_subtree(subtree, buffer, name, id, size)

	if size == 0 then
		return name
	end

	local count = buffer(0,1):uint()

	for i = 0,count-1 do
		local data = buffer(i*6+1,6)
		laser = subtree:add(data, "Laser " .. i)

		laser:add(data(0,1), "Type: " .. data(0,1):uint())
        laser:add(data(1,3), "Enabled: " .. data(1,3):bitfield(0,1))
        laser:add(data(1,3), "Armed: " .. data(1,3):bitfield(1,1))
        laser:add(data(1,3), "Active: " .. data(1,3):bitfield(2,1))
        laser:add(data(1,3), "Ground Speed Lock: " .. data(1,3):bitfield(3,1))
        laser:add(data(1,3), "Altitude Lock: " .. data(1,3):bitfield(4,1))
        laser:add(data(1,3), "Password Lock: " .. data(1,3):bitfield(5,1))
        laser:add(data(1,3), "AP Comm Lock: " .. data(1,3):bitfield(6,1))
        laser:add(data(1,3), "AP Flying Lock: " .. data(1,3):bitfield(7,1))
        laser:add(data(1,3), "Bypass Enabled: " .. data(1,3):bitfield(8,1))
        laser:add(data(1,3), "Pitch Angle Lock: " .. data(1,3):bitfield(9,1))

        laser:add(data(4,2), "Wait Timer: " .. data(4,2):uint())

	end

	return name
end

function print_diagnostics(subtree, buffer, id, size)
	local name = "Diagnostics"
	subtree = make_subtree(subtree, buffer, name, id, size)

	if size == 0 then
		return name
	end

	subtree:add(buffer(12,1), "Crown Temp: " .. buffer(12,1):uint() .. "º")
	subtree:add(buffer(13,1), "SLA Temp: " .. buffer(13,1):uint() .. "º")
	subtree:add(buffer(14,1), "Gyro Temp: " .. buffer(14,1):uint() .. "º")

	local v24 = subtree:add(buffer, "24 Vdc Statistics")
	v24:add(buffer(0,2), "Voltage: " .. buffer(0,2):uint() / 1000.0)
	v24:add(buffer(6,2), "Current: " .. buffer(6,2):uint() / 1000.0)
	v24:add(buffer(16,2), "RMS Voltage: " .. buffer(16,2):uint() / 1000000.0)
	v24:add(buffer(22,2), "RMS Current: " .. buffer(22,2):uint() / 1000000.0)

	local v12 = subtree:add(buffer, "12 Vdc Statistics")
	v12:add(buffer(2,2), "Voltage: " .. buffer(2,2):uint() / 1000.0)
	v12:add(buffer(8,2), "Current: " .. buffer(8,2):uint() / 1000.0)
	v12:add(buffer(18,2), "RMS Voltage: " .. buffer(18,2):uint() / 1000000.0)
	v12:add(buffer(24,2), "RMS Current: " .. buffer(24,2):uint() / 1000000.0)

	local v33 = subtree:add(buffer, "3.3 Vdc Statistics")
	v33:add(buffer(4,2), "Voltage: " .. buffer(4,2):uint() / 1000.0)
	v33:add(buffer(10,2), "Current: " .. buffer(10,2):uint() / 1000.0)
	v33:add(buffer(20,2), "RMS Voltage: " .. buffer(20,2):uint() / 1000000.0)
	v33:add(buffer(26,2), "RMS Current: " .. buffer(26,2):uint() / 1000000.0)

	if size > 28 then
		subtree:add(buffer(28,1), "Payload Temp: " .. buffer(28,1):int())
		subtree:add(buffer(29,1), "Payload Humidity: " .. buffer(29,1):uint() / 2.55)
	end

	return name
end

function print_sw_diagnostics(subtree, buffer, id, size)
	local board = buffer(0,1):uint()

	local name = get_board_string(board) .. " Board SW Diagnostics"
	subtree = make_subtree(subtree, buffer, name, id, size)

	if size == 0 then
		return name
	end

	local j = 4

	for i=1,buffer(1,1):uint() do
		local threads = math.min(buffer(j+4,1):uint()-1,9)
		local size = 5 + threads * 6
		local core = subtree:add(buffer(j, size), "Core " .. i - 1 .. " Loading")

		core:add(buffer(j+0,1), string.format("CPU Load: %.0f%%", buffer(j+0,1):uint() / 2.55))
		core:add(buffer(j+1,1), string.format("Heap Load: %.0f%%", buffer(j+1,1):uint() / 2.55))
		core:add(buffer(j+2,1), string.format("Stack Load: %.0f%%", buffer(j+2,1):uint() / 2.55))

		for k=0,threads do
			local thread = core:add(buffer(j + 5 + k * 6, 6), "Thread " .. k .. " Loading")
			local load = buffer(j + 5 + k * 6 + 0, 1):uint() / 255.0
			local iter = buffer(j + 5 + k * 6 + 4, 1):uint()

			thread:add(buffer(j + 5 + k * 6 + 0, 1), string.format("CPU Load: %.0f%%", buffer(j + 5 + k * 6 + 0, 1):uint() / 2.55))
			thread:add(buffer(j + 5 + k * 6 + 1, 1), string.format("Heap Load: %.0f%%", buffer(j + 5 + k * 6 + 1, 1):uint() / 2.55))
			thread:add(buffer(j + 5 + k * 6 + 2, 1), string.format("Stack Load: %.0f%%", buffer(j + 5 + k * 6 + 2, 1):uint() / 2.55))
			thread:add(buffer(j + 5 + k * 6 + 3, 1), string.format("WDT Left: %.0f%%", buffer(j + 5 + k * 6 + 3, 1):uint() / 2.55))
			thread:add(buffer(j + 5 + k * 6 + 4, 1), string.format("Iterations: %d", buffer(j + 5 + k * 6 + 4, 1):uint()))

			if iter > 0 and iter < 255 then
				local period = 5.0
				local average_ms = period * load / iter * 1000.0
				local worst = buffer(j + 5 + k * 6 + 5, 1):uint() / 10.0
				thread:add(buffer(j + 5 + k * 6, 6), string.format("Average Time: %dms", average_ms))
				thread:add(buffer(j + 5 + k * 6 + 5, 1), string.format("Worst Case (%.1fx): %.1fms", worst, average_ms * worst))
			end

		end

		j = j + size
	end

	return name

end	

function print_performance(subtree, buffer, id, size)
	local name = "Performance"
	subtree = make_subtree(subtree, buffer, name, id, size)

	if size == 0 then
		return name
	end

	subtree:add(buffer(0,2),  "Pan Quadrature Current Jitter: " .. buffer(0,2):uint() .. " uA")
	subtree:add(buffer(4,2),  "Pan Direct Current Jitter: " .. buffer(4,2):uint() .. " uA")
	subtree:add(buffer(8,2),  "Pan Velocity Jitter: " .. buffer(8,2):uint() .. " mrad/s")
	subtree:add(buffer(12,2), "Pan Position Jitter: " .. buffer(12,2):uint() .. " urad")
	subtree:add(buffer(16,2), "Pan Output Current: " .. buffer(16,2):uint() * 0.001 .. " Amps")
	subtree:add(buffer(2,2),  "Tilt Quadrature Current Jitter: " .. buffer(2,2):uint() .. " uA")
	subtree:add(buffer(6,2),  "Tilt Direct Current Jitter: " .. buffer(6,2):uint() .. " uA")
	subtree:add(buffer(10,2), "Tilt Velocity Jitter: " .. buffer(10,2):uint() .. " mrad/s")
	subtree:add(buffer(14,2), "Tilt Position Jitter: " .. buffer(14,2):uint() .. " urad")
	subtree:add(buffer(18,2), "Tilt Output Current: " .. buffer(18,2):uint() * 0.001 .. " Amps")

	return name
end

function print_retract_cmd(subtree, buffer, id, size)
    local name = "Retract Cmd"
    subtree = make_subtree(subtree, buffer, name, id, size)

    if size == 0 then
        return name
    end

    local cmd_enum = buffer(0,1):bitfield(1,7)
    local cmd_string = "None"

    if cmd_enum == 0 then cmd_string = "Disable"
    elseif cmd_enum == 1 then cmd_string = "Deploy"
    elseif cmd_enum == 2 then cmd_string = "Retract"
    end

    subtree:add(buffer(0,1), "Cmd: " .. cmd_string)

    return name
end

function print_retract_status(subtree, buffer, id, size)
    local name = "Retract Status"
    subtree = make_subtree(subtree, buffer, name, id, size)

    if size == 0 then
        return name
    end

    local cmd_enum = buffer(0,1):bitfield(1,7)
    local cmd_string = "None"

    if cmd_enum == 0 then cmd_string = "Disable"
    elseif cmd_enum == 1 then cmd_string = "Deploy"
    elseif cmd_enum == 2 then cmd_string = "Retract"
    end

    local state_enum = buffer(1,2):bitfield(1,7)
    local state_string = "Disabled"

    if state_enum == 0 then state_string = "Disabled"
    elseif state_enum == 1 then state_string = "Retracted"
    elseif state_enum == 2 then state_string = "Retracting"
    elseif state_enum == 3 then state_string = "Deploying"
    elseif state_enum == 4 then state_string = "Deployed"
    elseif state_enum == 5 then state_string = "FAULT"
    end

    subtree:add(buffer(0,1), "Cmd: " .. cmd_string)
    subtree:add(buffer(1,2), "State: " .. state_string)
    subtree:add(buffer(2,2), string.format("Pos: %.2f deg", math.deg(buffer(2,2):int() / 1000)) )
    subtree:add(buffer(4,2), "Flags: " .. buffer(4,2):uint())

    return name
end

function print_gps_data(subtree, buffer, id, size)
	local name = "GPS Data"
	subtree = make_subtree(subtree, buffer, name, id, size)

	if size == 0 then
		return name
	end

    local fix_type = buffer(0,1):bitfield(1,7)
	local fix_string = "None"
	local vertical_valid = buffer(0,1):bitfield(0,1)

	if fix_type == 1 then fix_string = "Dead Reckoning"
	elseif fix_type == 2 then fix_string = "2D"
	elseif fix_type == 3 then fix_string = "3D"
	elseif fix_type == 4 then fix_string = "GNSS Dead Reckoning"
	elseif fix_type == 5 then fix_string = "Time Only"
	end

	subtree:add(buffer(0,1), "Multi Ant. Heading Valid: " .. vertical_valid)
    subtree:add(buffer(0,1), "Fix Type: " .. fix_string)
    subtree:add(buffer(1,1), "Fix State: " .. buffer(1,1):uint())
    subtree:add(buffer(2,1), "Satellites: " .. buffer(2,1):uint())
    subtree:add(buffer(3,1), "PDOP: " .. buffer(3,1):uint() * 0.1)
	subtree:add(buffer(4,4), "Latitude: " .. buffer(4,4):int() / 10000000.0 .. "º")
	subtree:add(buffer(8,4), "Longitude: " .. buffer(8,4):int() / 10000000.0 .. "º")
	subtree:add(buffer(12,4), "Altitude: " .. buffer(12,4):int() / 10000.0 .. "m")
	subtree:add(buffer(16,4), "Vel North: " .. buffer(16,4):int() / 1000.0 .. " m/s")
	subtree:add(buffer(20,4), "Vel East: " .. buffer(20,4):int() / 1000.0 .. " m/s")
	subtree:add(buffer(24,4), "Vel Down: " .. buffer(24,4):int() / 1000.0 .. " m/s")

	local acc = subtree:add(buffer(28,16), "Accuracy")
	acc:add(buffer(28,4), "Horiz. Accuracy: " .. buffer(28,4):int() / 1000.0 .. "m")
	acc:add(buffer(32,4), "Vert. Accuracy: " .. buffer(32,4):int() / 1000.0 .. "m")
	acc:add(buffer(36,4), "Speed Accuracy: " .. buffer(36,4):int() / 1000.0 .. " m/s")
	acc:add(buffer(40,4), "Hdg. Accuracy: " .. buffer(40,4):int() / 100000.0 .. "º")

	subtree:add(buffer(44,4), "ITOW: " .. buffer(44,4):uint())
	subtree:add(buffer(48,2), "GPS Week: " .. buffer(48,2):uint())
	subtree:add(buffer(50,2), "Geoid Undulation: " .. buffer(50,2):int() / 100.0 .. "m")

	subtree:add(buffer(52,1), "Source: " .. get_gps_string(buffer(52,1):uint()))

	if size >= 54 then
		subtree:add(buffer(53,1), "Vertical Velocity Valid: " .. buffer(53,1):uint())
	end

	if size >= 55 then
		subtree:add(buffer(54,1), "Leap Seconds: " .. buffer(54,1):uint())
	end

	if size >= 57 then
		subtree:add(buffer(55,2), "Multi Ant. Heading: " .. buffer(55,2):int() / 32768.0 * 180)
	end

	return name
end

function print_ext_heading(subtree, buffer, id, size)
	local name = "Ext. Heading"
	subtree = make_subtree(subtree, buffer, name, id, size)

	if size == 0 then
		return name
	end

	subtree:add(buffer(0,2), "Heading: " .. buffer(0,2):int() / 32768.0 * 180.0)
	subtree:add(buffer(2,2), "Noise: " .. buffer(2,2):uint() / 32768.0 * 360.0)

	if size >= 5 then
		subtree:add(buffer(4,1), "Bitfield: " .. buffer(4,1))
	end

	return name
end

function print_imu_data_short(subtree, buffer, id, size)
	local name = "IMU Data Short"
	subtree = make_subtree(subtree, buffer, name, id, size)

	if size == 0 then
		return name
	end

	subtree:add(orion_systime, buffer(0,4):uint())

	return name
end

function print_ins_quality(subtree, buffer, id, size)
	local name = "INS Quality"
	subtree = make_subtree(subtree, buffer, name, id, size)

	if size == 0 then
		return name
	end

	subtree:add(buffer(0,4), "System Time: " .. buffer(0,4):uint())
	subtree:add(buffer(4,1), "GPS Source: " .. get_gps_string(buffer(4,1):uint()))

	local imu_type = buffer(5,1):bitfield(6,2)
	local imu_string = "Other"

	if imu_type == 0 then imu_string = "Internal"
	elseif imu_type == 1 then imu_string = "Sensonor"
	elseif imu_type == 2 then imu_string = "DMU-11"
	end

	subtree:add(buffer(5,1), "IMU Type: " .. imu_string)

	local imu_mode = buffer(6,1):uint()

	if imu_mode == 0 then imu_string = "Init 1"
	elseif imu_mode == 1 then imu_string = "Init 2"
	elseif imu_mode == 2 then imu_string = "AHRS"
	elseif imu_mode == 3 then imu_string = "Run Hard"
	elseif imu_mode == 4 then imu_string = "Run"
	elseif imu_mode == 5 then imu_string "Run TC"
	end

	subtree:add(buffer(6,1), "INS Mode: " .. imu_string)

	-- Skipping because it looks as though the scaling is broken...
	-- subtree:add(buffer(8,1), "GPS Period" .. buffer(8,1):uint() / 100.0)
	-- subtree:add(buffer(9,1), "Heading Period" .. buffer(9,1):uint() / 100.0)

	-- Skipping because I don't have a float16 to float32 conversion
	-- local chi = subtree:add(buffer(10,6), "Chi-Square Statistics")
	-- chi:add(buffer(10,2), "Position" .. buffer(10,2):uint())
	-- chi:add(buffer(12,2), "Velocity" .. buffer(12,2):uint())
	-- chi:add(buffer(14,2), "Heading" .. buffer(14,2):uint())

	local att = subtree:add(buffer(16,6), "Att. Confidence")
	att:add(buffer(16,2), "Roll: " .. math.deg(buffer(16,2):uint() / 10000.0))
	att:add(buffer(18,2), "Pitch: " .. math.deg(buffer(18,2):uint() / 10000.0))
	att:add(buffer(20,2), "Yaw: " .. math.deg(buffer(20,2):uint() / 10000.0))

	local vel = subtree:add(buffer(22,6), "Vel. Confidence")
	vel:add(buffer(22,2), "North: " .. buffer(22,2):uint() / 100.0)
	vel:add(buffer(24,2), "East: " .. buffer(24,2):uint() / 100.0)
	vel:add(buffer(26,2), "Down: " .. buffer(26,2):uint() / 100.0)

	local pos = subtree:add(buffer(28,6), "Pos. Confidence")
	pos:add(buffer(28,2), "X: " .. buffer(28,2):uint() / 100.0)
	pos:add(buffer(30,2), "Y: " .. buffer(30,2):uint() / 100.0)
	pos:add(buffer(32,2), "Z: " .. buffer(32,2):uint() / 100.0)

	local i = 34

	if buffer(7,1):bitfield(0,1) == 1 and size >= i + 6 then
		local pos = subtree:add(buffer(i,6), "Gyro Bias Confidence")
		pos:add(buffer(i+0,2), "p: " .. buffer(i+0,2):uint() / 100000.0)
		pos:add(buffer(i+2,2), "q: " .. buffer(i+2,2):uint() / 100000.0)
		pos:add(buffer(i+4,2), "r: " .. buffer(i+4,2):uint() / 100000.0)
		i = i + 6
	end

	if buffer(7,1):bitfield(2,1) == 1 and size >= i + 6 then
		local pos = subtree:add(buffer(i,6), "Accel Bias Confidence")
		pos:add(buffer(i+0,2), "X: " .. buffer(i+0,2):uint() / 30000.0)
		pos:add(buffer(i+2,2), "Y: " .. buffer(i+2,2):uint() / 30000.0)
		pos:add(buffer(i+4,2), "Z: " .. buffer(i+4,2):uint() / 30000.0)
		i = i + 6
	end

	if buffer(7,1):bitfield(1,1) == 1 and size >= i + 2 then
		subtree:add(buffer(i,2), "Gravity Bias Confidence: " .. buffer(i,2):uint() / 30000)
		i = i + 2
	end

	if buffer(7,1):bitfield(3,1) == 1 and size >= i + 4 then
		subtree:add(buffer(i+0,2), "Clock Bias Confidence: " .. buffer(i+0,2):uint() / 10000)
		subtree:add(buffer(i+2,2), "Clock Drift Confidence: " .. buffer(i+2,2):uint() / 10000)
		i = i + 4
	end

	if buffer(7,1):bitfield(0,1) == 1 and size >= i + 6 then
		local pos = subtree:add(buffer(i,6), "Gyro Bias")
		pos:add(buffer(i+0,2), "p: " .. math.deg(buffer(i+0,2):int() / 100000.0))
		pos:add(buffer(i+2,2), "q: " .. math.deg(buffer(i+2,2):int() / 100000.0))
		pos:add(buffer(i+4,2), "r: " .. math.deg(buffer(i+4,2):int() / 100000.0))
		i = i + 6
	end

	if buffer(7,1):bitfield(2,1) == 1 and size >= i + 6 then
		local pos = subtree:add(buffer(i,6), "Accel Bias")
		pos:add(buffer(i+0,2), "X: " .. buffer(i+0,2):uint() / 30000.0)
		pos:add(buffer(i+2,2), "Y: " .. buffer(i+2,2):uint() / 30000.0)
		pos:add(buffer(i+4,2), "Z: " .. buffer(i+4,2):uint() / 30000.0)
		i = i + 6
	end

	if buffer(7,1):bitfield(1,1) == 1 and size >= i + 2 then
		subtree:add(buffer(i,2), "Gravity Bias: " .. buffer(i,2):int() / 30000)
		i = i + 2
	end

	if buffer(7,1):bitfield(3,1) == 1 and size >= i + 8 then
		subtree:add(buffer(i+0,2), "Clock Bias" .. buffer(i+0,2):int() / 100)
		subtree:add(buffer(i+2,2), "Clock Drift" .. buffer(i+2,2):int() / 1000)
		subtree:add(buffer(i+4,1), "TC Sat Pos Updates: " .. buffer(i+4,1):uint())
		subtree:add(buffer(i+5,1), "TC Sat Vel Updates: " .. buffer(i+5,1):uint())
		subtree:add(buffer(i+6,1), "TC Pos Updates: " .. buffer(i+6,1):uint())
		subtree:add(buffer(i+7,1), "TC Vel Updates: " .. buffer(i+7,1):uint())
		i = i + 8
	end

	return name
end

function print_geolocate(subtree, buffer, id, size)
	local name = "Geolocate Telemetry"
	subtree = make_subtree(subtree, buffer, name, id, size)

	if size == 0 then
		return name
	end

	subtree:add(buffer(0,4), "System Time: " .. buffer(0,4):uint())
	subtree:add(buffer(4,4), "GPS Time of Week: " .. buffer(4,4):uint())
	subtree:add(buffer(8,2), "Gps Week: " .. buffer(8,2):uint())
	subtree:add(buffer(10,2), string.format("Geoid Undulation: %.1f", buffer(10,2):int() / 32768.0 * 120.0))
	subtree:add(buffer(12,4), string.format("Latitude: %.6f", buffer(12,4):int() / 10000000.0))
	subtree:add(buffer(16,4), string.format("Longitude: %.6f", buffer(16,4):int() / 10000000.0))
	subtree:add(buffer(20,4), string.format("Altitude: %.1f", buffer(20,4):int() / 10000.0))

	local gps_vel = subtree:add(buffer(24,6), "GPS Velocity")

	gps_vel:add(buffer(24,2), "North: " .. buffer(24,2):int() / 100.0)
	gps_vel:add(buffer(26,2), "East: " .. buffer(26,2):int() / 100.0)
	gps_vel:add(buffer(28,2), "Down: " .. buffer(28,2):int() / 100.0)

	local a = subtree:add(buffer(30,8), "Gimbal Attitude")

	local euler = quat_to_euler(buffer(30,2):int() / 32768.0,
								buffer(32,2):int() / 32768.0,
								buffer(34,2):int() / 32768.0,
								buffer(36,2):int() / 32768.0)

	a:add(buffer(30,8), string.format("Roll: %.2f", math.deg(euler[0])))
	a:add(buffer(30,8), string.format("Pitch: %.2f", math.deg(euler[1])))
	a:add(buffer(30,8), string.format("Yaw: %.2f", math.deg(euler[2])))

	subtree:add(buffer(38,2), string.format("Pan: %.1f", buffer(38,2):int() / 32768.0 * 180.0))
	subtree:add(buffer(40,2), string.format("Tilt: %.1f", buffer(40,2):int() / 32768.0 * 180.0))

	subtree:add(buffer(42,2), string.format("HFOV: %.1f", buffer(42,2):uint() / 65535.0 * 360.0))
	subtree:add(buffer(44,2), string.format("VFOV: %.1f", buffer(44,2):uint() / 65535.0 * 360.0))

	if size >= 52 then
		local los = subtree:add(buffer(46,6), "Target Position")
		local lat = math.rad(buffer(12,4):int() / 10000000.0)
		local lon = math.rad(buffer(16,4):int() / 10000000.0)
		local alt = buffer(20,4):int() / 10000.0

		local gimbal_ecef = lla_to_ecef(lat, lon, alt)

		local x = buffer(46,2):int()
		local y = buffer(48,2):int()
		local z = buffer(50,2):int()

		lla = ecef_to_lla(x + gimbal_ecef[0], y + gimbal_ecef[1], z + gimbal_ecef[2])

		los:add(buffer(46,6), string.format("Latitude: %.6f", math.deg(lla[0])))
		los:add(buffer(46,6), string.format("Longitude: %.6f", math.deg(lla[1])))
		los:add(buffer(46,6), string.format("Altitude: %.1f", lla[2]))
		los:add(buffer(46,6), string.format("Slant Range: %.1f", math.sqrt(x*x+y*y+z*z)))
	end

	if size >= 54 then
		subtree:add(buffer(52,2), "Pixel Width: " .. buffer(52,2):uint())
	end

	if size >= 56 then
		subtree:add(buffer(54,2), "Pixel Height: " .. buffer(54,2):uint())
	end

	if size >= 57 then
		subtree:add(buffer(56,1), "Mode: " .. get_mode_string(buffer(56,1):uint()))
	end

	if size >= 61 then

		local path = subtree:add(buffer(57,4), "Path Status")

		path:add(buffer(57,1), "Path Progress: " .. buffer(57,1):uint() / 255.0)

		if size >= 59 then
			path:add(buffer(58,1), "Stare Time: " .. buffer(58,1):uint() / 100.0)
		end

		if size >= 60 then
			path:add(buffer(59,1), "Path From: " .. buffer(59,1):uint())
		end

		if size >= 61 then
			path:add(buffer(60,1), "Path To: " .. buffer(60,1):uint())
		end
	end

	if size >= 76 then
		local shifts = subtree:add(buffer(61,15), "Image Shift Data")

		shifts:add(buffer(61,4), "Raw Shift X: " .. math.deg(buffer(61,4):int() / 1000000.0))
		shifts:add(buffer(65,4), "Raw Shift Y: " .. math.deg(buffer(65,4):int() / 1000000.0))
		shifts:add(buffer(69,2), "Delta Time: " .. buffer(69,2):uint() / 1000.0)
		shifts:add(buffer(71,1), "Confidence: " .. buffer(71,1):uint() / 2.55 .. "%")
		shifts:add(buffer(72,2), "Output Shift X: " .. buffer(72,2):int() / 32768.0 * 180.0)
		shifts:add(buffer(74,2), "Output Shift Y: " .. buffer(74,2):int() / 32768.0 * 180.0)
	end

	if size >= 77 then
		subtree:add(buffer(76,1), "Range Source: " .. get_range_src_string(buffer(76,1):uint()))
	end

	if size >= 78 then
		subtree:add(buffer(77,1), "Leap Seconds: " .. buffer(77,1):uint())
	end

	return name

end

function print_geopoint_cmd(subtree, buffer, id, size)
	local name = "Geopoint Cmd"
	subtree = make_subtree(subtree, buffer, name, id, size)

	if size == 0 then
		return name
	end

	subtree:add(buffer(0,4), string.format("Latitude: %.6f", buffer(0,4):int() / 10000000.0))
	subtree:add(buffer(4,4), string.format("Longitude: %.6f", buffer(4,4):int() / 10000000.0))
	subtree:add(buffer(8,4), string.format("Altitude: %.1f", buffer(8,4):int() / 10000.0))


	local vel = subtree:add(buffer(12,6), "Velocity")

	vel:add(buffer(12,2), "North: " .. buffer(12,2):int() / 100.0)
	vel:add(buffer(14,2), "East: " .. buffer(14,2):int() / 100.0)
	vel:add(buffer(16,2), "Down: " .. buffer(16,2):int() / 100.0)

	return name
end

function print_network_video(subtree, buffer, id, size)
	local name = "Network Video"
	subtree = make_subtree(subtree, buffer, name, id, size)

	if size == 0 then
		return name
	end

	subtree:add(buffer(0,4), "Destination IP: " .. get_ip_string(buffer(0,4)))
	subtree:add(buffer(4,2), "Port: " .. buffer(4,2):uint())
	subtree:add(buffer(6,4), "Bitrate: " .. buffer(6,4):uint())

	if size >= 11 then
		subtree:add(buffer(10,1), "TTL: " .. buffer(10,1):int())
	end

	if size >= 12 then
		local stream_string = "Stream Type: "
		local stream_val = buffer(11,1):int()

		if stream_val == 0 then
			stream_string = stream_string .. "h.264"
		elseif stream_val == 1 then
			stream_string = stream_string .. "MJPEG"
		elseif stream_val == 2 then
			stream_string = stream_string .. "RAW"
		elseif stream_val == 3 then
			stream_string = stream_string .. "YUV"
		else
			stream_string = stream_string .. "Unknown"
		end

		subtree:add(buffer(11,1), stream_string)
	end

	if size >= 13 then
		subtree:add(buffer(12,1), "MJPEG Quality: " .. buffer(12,1):int())
	end

	return name
end

function print_version(subtree, buffer, id, size)
	local name = " Version"
	local len = 16

	if id == 37 then 
		name = "Clevis" .. name
	elseif  id == 40 then
		name = "Crown" .. name
	elseif id == 41 then
		name = "Payload" .. name
		len = 24
	elseif id == 44 then
		name = "Tracker" .. name	
	end

	subtree = make_subtree(subtree, buffer, name, id, size)

	if size == 0 then
		return name
	end

	subtree:add(buffer(0,len), "Version: " .. buffer(0,len):string(ENC_UTF_8))
	subtree:add(buffer(len,16), "Part Number: " .. buffer(len,16):string(ENC_UTF_8))

	if size >= len + 17 then
		subtree:add(buffer(len+16,4), "On Time: " .. buffer(len+16,4):uint())
	end

	return name

end


function print_network_diagnostics(subtree, buffer, id, size)
	local name = "Network Diagnostics"
	subtree = make_subtree(subtree, buffer, name, id, size)

	if size == 0 then
		return name
	end

    subtree:add(buffer(0,2), "Flags: " .. buffer(0,2):uint())
    subtree:add(buffer(2,4), "Rx Bytes: " .. buffer(2,4):uint())
    subtree:add(buffer(6,4), "Tx Bytes: " .. buffer(6,4):uint())
    subtree:add(buffer(10,4), "Rx Packets: " .. buffer(10,4):uint())
    subtree:add(buffer(14,4), "Tx Packets: " .. buffer(14,4):uint())
    subtree:add(buffer(18,2), "Rx Errors: " .. buffer(18,2):uint())
    subtree:add(buffer(20,2), "Tx Errors: " .. buffer(20,2):uint())
    subtree:add(buffer(22,2), "Rx Drops: " .. buffer(22,2):uint())
    subtree:add(buffer(24,2), "Tx Drops: " .. buffer(24,2):uint())
    subtree:add(buffer(26,2), "Rx FIFO Errors: " .. buffer(26,2):uint())
    subtree:add(buffer(28,2), "Tx FIFO Errors: " .. buffer(28,2):uint())
    subtree:add(buffer(30,2), "Frame Errors: " .. buffer(30,2):uint())
    subtree:add(buffer(32,2), "Collisions: " .. buffer(32,2):uint())
    subtree:add(buffer(34,2), "Carrier Errors: " .. buffer(34,2):uint())

    return name

end

function print_range(subtree, buffer, id, size)
	local name = "Range Data"
	subtree = make_subtree(subtree, buffer, name, id, size)

	if size == 0 then
		return name
	end

	subtree:add(buffer(0,4), "Range [m]: " .. buffer(0,4):uint() * 0.01)
	subtree:add(buffer(4,2), "Max Age [ms]: " .. buffer(4,2):uint())
	subtree:add(buffer(6,1), "Range Source: " .. get_range_src_string(buffer(6,1):uint()))

	return name

end

function to_uint24(buffer)
	return buffer:bitfield(0,24)
end

function to_int24(buffer)
	local value = to_uint24(buffer)

	if value >= 0x800000 then
		return 0x1000000 - value
	else
		return value
	end
end


function print_path_data(subtree, buffer, id, size)
	local name = "Path Data"
	subtree = make_subtree(subtree, buffer, name, id, size)

	if size == 0 then
		return name
	end

	subtree:add(buffer(0,1), "Number of Points: " .. buffer(0,1):uint())

	local j = 2

	-- Notably missing: pointDown and numCrossTrackSteps

	for i=1,buffer(0,1):uint() do

		local p = subtree:add(buffer(j,9), "Point " .. (i - 1))

		local x = to_int24(buffer(j+0,3))
		local y = to_int24(buffer(j+3,3))
		local z = to_int24(buffer(j+6,3))

		local lla = ecef_to_lla(x, y, z)

		p:add(buffer(j+0,3), string.format("Latitude: %.6f", math.deg(lla[0])))
		p:add(buffer(j+3,3), string.format("Longitude: %.6f", math.deg(lla[1])))
		p:add(buffer(j+6,3), string.format("Height: %.1f", lla[2]))

		j = j+9

	end

	if size >= j + 2 then
		subtree:add(buffer(j+0,2), "Along Track Angle: " .. buffer(j+0,2):uint() * 180.0 / 65535.0)
	end

	if size >= j + 3 then
		subtree:add(buffer(j+2,1), "Cross Track Ratio: " .. buffer(j+2,1):uint() * 0.01)
	end

	return name

end

function print_stare_start(subtree, buffer, id, size)
	local name = "Stare Start"
	subtree = make_subtree(subtree, buffer, name, id, size)

	if size == 0 then
		return name
	end

	subtree:add(buffer(0,4), "System Time: " .. buffer(0,4):uint())
	subtree:add(buffer(4,1), "Max Stare Time: " .. buffer(4,1):uint() * 0.01)
	subtree:add(buffer(5,1), "Along Track Stare: " .. buffer(5,1):bitfield(0,1))
	subtree:add(buffer(5,1), "Previous Stare Timed Out: " .. buffer(5,1):bitfield(1,1))
	subtree:add(buffer(5,1), "Stare Started by Time: " .. buffer(5,1):bitfield(2,1))
	subtree:add(buffer(5,1), "Gimbal Data Valid: " .. buffer(5,1):bitfield(3,1))
	subtree:add(buffer(5,1), "Stare Reset: " .. buffer(5,1):bitfield(4,1))
	subtree:add(buffer(6,1), "Previous Stare Time: " .. buffer(6,1):uint() * 0.01)
	subtree:add(buffer(7,1), "Stare Load: " .. buffer(7,1):uint() / 2.55 .. "%")


	local stare = subtree:add(buffer(8,12), "Stare Position")
	stare:add(buffer(8,4), "Latitude: " .. buffer(8,4):int() / 10000000.0)
	stare:add(buffer(12,4), "Longitude: " .. buffer(12,4):int() / 10000000.0)
	stare:add(buffer(16,4), "Altitude: " .. buffer(16,4):int() / 10000.0)


	local gimbal = subtree:add(buffer(20,12), "Gimbal Position")
	gimbal:add(buffer(20,4), "Latitude: " .. buffer(20,4):int() / 10000000.0)
	gimbal:add(buffer(24,4), "Longitude: " .. buffer(24,4):int() / 10000000.0)
	gimbal:add(buffer(28,4), "Altitude: " .. buffer(28,4):int() / 10000.0)

	local att = subtree:add(buffer(32,8), "Gimbal Attitude")

	local euler = quat_to_euler(buffer(32,2):int() / 32768.0,
								buffer(34,2):int() / 32768.0,
								buffer(36,2):int() / 32768.0,
								buffer(38,2):int() / 32768.0)

	att:add(buffer(32,8), string.format("Roll: %.2f", math.deg(euler[0])))
	att:add(buffer(32,8), string.format("Pitch: %.2f", math.deg(euler[1])))
	att:add(buffer(32,8), string.format("Yaw: %.2f", math.deg(euler[2])))

	if size >= 44 then
		subtree:add(buffer(40,2), string.format("Pan: %.1f", buffer(40,2):int() / 32768.0 * 180.0))
		subtree:add(buffer(42,2), string.format("Tilt: %.1f", buffer(42,2):int() / 32768.0 * 180.0))
	end

	if size >= 48 then
		subtree:add(buffer(44,1), "Range Source: " .. get_range_src_string(buffer(44,1):uint()))
		subtree:add(buffer(45,3), "Slant Range: " .. to_uint24(buffer(45,3)) / 100.0)
	end

	if size >= 52 then
		subtree:add(buffer(48,2), string.format("Along Stare Angle: %.1f", buffer(48,2):int() / 32768.0 * 180.0))
		subtree:add(buffer(50,2), string.format("Cross Stare Angle: %.1f", buffer(50,2):int() / 32768.0 * 180.0))
	end

	return name

end

function print_stare_ack(subtree, buffer, id, size)
	local name = "Stare Ack"
	subtree = make_subtree(subtree, buffer, name, id, size)

	if size == 0 then
		return name
	end

	subtree:add(buffer(0,4), "System Time: " .. buffer(0,4):uint())

	return name

end

function print_board_info(subtree, buffer, id, size)
	local name = "Board Info"
	subtree = make_subtree(subtree, buffer, name, id, size)

	if size == 0 then
		return name
	end

	subtree:add(buffer(0,4), "IMU Serial Number: " .. buffer(0,4):uint())
	subtree:add(buffer(4,4), "Gimbal Serial Number: " .. buffer(4,4):uint())
	subtree:add(buffer(8,4), "Configuration Bits: 0x" .. buffer(8,4))

	local date_int = buffer(16,2):uint()

	local y = buffer(16,2):bitfield(0,7) + 2000
	local m = buffer(16,2):bitfield(7,4)
	local d = buffer(16,2):bitfield(11,5)

	subtree:add(buffer(16,2), string.format("Manufacture Date: %4d/%02d/%02d", y, m, d))

	local date_int = buffer(18,2):uint()

	local y = buffer(16,2):bitfield(0,7) + 2000
	local m = buffer(16,2):bitfield(7,4)
	local d = buffer(16,2):bitfield(11,5)

	subtree:add(buffer(18,2), string.format("Calibration Date: %4d/%02d/%02d", y, m, d))

	return name
end

function print_camera_state(subtree, buffer, id, size)
	local name = "Camera State"
	subtree = make_subtree(subtree, buffer, name, id, size)

	if size == 0 then
		return name
	end

	subtree:add(buffer(0,2), "Zoom: " .. buffer(0,2):int() * 0.01)
	subtree:add(buffer(2,2), "Focus: " .. buffer(2,2):int() * 0.0001)
	subtree:add(buffer(4,1), "Index: " .. buffer(4,1):uint())

	return name

end

function print_cameras(subtree, buffer, id, size)
	local name = "Cameras"
	subtree = make_subtree(subtree, buffer, name, id, size)

	if size == 0 then
		return name
	end

	local int cameras = buffer(0,1):uint()

	subtree:add(buffer(0,1), "Number of Cameras: " .. cameras)

	for i=0,cameras-1 do
		local cam = subtree:add(buffer(24*i+4,24), "Camera " .. i)
		local data = buffer(24*i+4,24)

		local type_int = data(0,1):uint()
		local type_str = "None"

		if type_int == 1 then
			type_str = "Visible"
		elseif type_int == 2 then
			type_str = "SWIR"
		elseif type_int == 3 then
			type_str = "MWIR"
		elseif type_int == 4 then
			type_str = "LWIR"
		end

		cam:add(data(0,1), "Type: " .. type_str)

		local proto_int = data(1,1):uint()
		local proto_str = "Unknown"

		if proto_int == 1 then
			proto_str = "FLIR"
		elseif proto_int == 2 then
			proto_str = "Aptina"
		elseif proto_int == 3 then
			proto_str = "Zafiro"
		elseif proto_int == 4 then
			proto_str = "Hitachi"
		elseif proto_int == 5 then
			proto_str = "BAE"
		elseif proto_int == 6 then
			proto_str = "Sony"
		end

		cam:add(data(1,1), "Protocol: " .. proto_str)
		cam:add(data(2,4), string.format("Min Focal Length: %.0fmm", data(2,4):uint() / 1000.0))
		cam:add(data(6,4), string.format("Max Focal Length: %.0fmm", data(6,4):uint() / 1000.0))
		cam:add(data(10,2), string.format("Pixel Pitch: %.2fum", data(10,2):uint() / 1000.0))
		cam:add(data(12,2), string.format("Array Width: %u", data(12,2):uint()))
		cam:add(data(14,2), string.format("Array Height: %u", data(14,2):uint()))
		cam:add(data(16,2), string.format("Pan Offset 0: %.2f", data(16,2):int() / 32768.0 * 180))
		cam:add(data(18,2), string.format("Tilt Offset 0: %.2f", data(18,2):int() / 32768.0 * 180))
		cam:add(data(20,2), string.format("Pan Offset 1: %.2f", data(20,2):int() / 32768.0 * 180))
		cam:add(data(22,2), string.format("Tilt Offset 1: %.2f", data(22,2):int() / 32768.0 * 180))
	end

    return name
end



function print_network_settings(subtree, buffer, id, size)
	local name = "Network Settings"
	subtree = make_subtree(subtree, buffer, name, id, size)

	if size == 0 then
		return name
	end

	subtree:add(buffer(0,4), "IP Address: " .. get_ip_string(buffer(0,4):uint()))
	subtree:add(buffer(4,4), "Subnet Mask: " .. get_ip_string(buffer(4,4):uint()))
	subtree:add(buffer(8,4), "Gateway: " .. get_ip_string(buffer(8,4):uint()))

	if size >= 15 then
		subtree:add(buffer(13,2), "MTU: " .. buffer(13,2):uint())
	end

    return name
end


function print_packet(pinfo, subtree, buffer)
	local id = buffer(2,1):uint()
	local size = buffer(3,1):uint()
	local data = buffer(4,size)
	local info = ""

	if     id == 1   then info = print_cmd(subtree, data, id, size)
	elseif id == 5   then info = print_lasers(subtree, data, id, size)
	elseif id == 6   then info = print_laser_states(subtree, data, id, size)
	elseif id == 37  then info = print_version(subtree, data, id, size)
	elseif id == 39  then info = print_board_info(subtree, data, id, size)
	elseif id == 40  then info = print_version(subtree, data, id, size)
	elseif id == 41  then info = print_version(subtree, data, id, size)
	elseif id == 44  then info = print_version(subtree, data, id, size)
	elseif id == 65  then info = print_diagnostics(subtree, data, id, size)
	elseif id == 67  then info = print_performance(subtree, data, id, size)
	elseif id == 70  then info = print_network_diagnostics(subtree, data, id, size)
	elseif id == 68  then info = print_sw_diagnostics(subtree, data, id, size)
	elseif id == 97  then info = print_camera_state(subtree, data, id, size)
	elseif id == 98  then info = print_network_video(subtree, data, id, size)
	elseif id == 99  then info = print_cameras(subtree, data, id, size)
    elseif id == 160 then info = print_retract_cmd(subtree, data, id, size)
    elseif id == 161 then info = print_retract_status(subtree, data, id, size)
    -- elseif id == 177 then info = print_debug_string(subtree, data, id, size)
    elseif id == 209 then info = print_gps_data(subtree, data, id, size)
	elseif id == 210 then info = print_ext_heading(subtree, data, id, size)
	elseif id == 211 then info = print_ins_quality(subtree, data, id, size)
	elseif id == 212 then info = print_geolocate(subtree, data, id, size)
	elseif id == 213 then info = print_geopoint_cmd(subtree, data, id, size)
	elseif id == 214 then info = print_range(subtree, data, id, size)
	elseif id == 215 then info = print_path_data(subtree, data, id, size)
	elseif id == 217 then info = print_stare_start(subtree, data, id, size)
	elseif id == 218 then info = print_stare_ack(subtree, data, id, size)
	elseif id == 228 then info = print_network_settings(subtree, data, id, size)
	else
		local name = "Unknown packet ID 0x" .. buffer(2,1)
		subtree = make_subtree(subtree, data, name, id, size)
		subtree:add(data, "Data: " .. data)
		info = name
	end

	if tostring(pinfo.cols.info) == "" then
		pinfo.cols.info = info
	else
		pinfo.cols.info = tostring(pinfo.cols.info) .. ", " .. info
	end

	return size + 6
end

-- create a function to dissect it
function orion.dissector(buffer,pinfo,tree)
	if buffer(0,2):uint() == 53261 then

	    pinfo.cols.protocol = "Orion"

	    local i = 0
	    local bytes = buffer:len()

	    pinfo.cols.info = ""

	    while i < bytes do
	    	i = i + print_packet(pinfo, tree, buffer(i, bytes - i))
	    end
	end
end

DissectorTable.get("udp.port"):add(8745,orion)
DissectorTable.get("udp.port"):add(8746,orion)
DissectorTable.get("tcp.port"):add(8747,orion)
DissectorTable.get("udp.port"):add(8748,orion)
