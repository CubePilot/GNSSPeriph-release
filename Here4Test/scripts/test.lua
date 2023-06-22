--[[
Script to control LED strips based on the roll of the aircraft. This is an example to demonstrate
the LED interface for WS2812 LEDs
--]]

--[[
for this demo we will use a single strip with 30 LEDs
--]]
local num_leds = 4

--[[
 use SERVOn_FUNCTION 94 for LED. We can control up to 16 separate strips of LEDs
 by putting them on different channels
--]]
local chan = 2 --SRV_Channels:find_channel(94)
local last_change_ms = 0
local led = 0

if not chan then
    gcs:send_text(6, "LEDs: channel not set")
    return
end

gcs:send_text(6, "LEDs: chan=" .. tostring(chan))

-- initialisation code
--serialLED:set_num_neopixel(chan,  num_leds)
serialLED:set_num_profiled(chan,  num_leds)

-- constrain a value between limits
function constrain(v, vmin, vmax)
   if v < vmin then
      v = vmin
   end
   if v > vmax then
      v = vmax
   end
   return v
end

--[[
Table of colors on a rainbow, red first
--]]
local light_list = {
  { 0, 255, 0, 0 },
  { 0, 0,   255, 0 },
  { 0, 0,   0,   255 },
  { 1, 255, 0, 0 },
  { 1, 0,   255, 0 },
  { 1, 0,   0,   255 },
  { 2, 255, 0, 0 },
  { 2, 0,   255, 0 },
  { 2, 0,   0,   255 },
  { 3, 255, 0, 0 },
  { 3, 0,   255, 0 },
  { 3, 0,   0,   255 },
}

local list_index = 1

--[[
We will set the colour of the LEDs based on roll of the aircraft
--]]
function update_LEDs()
    serialLED:set_RGB(chan, light_list[list_index][1], light_list[list_index][2], light_list[list_index][3], light_list[list_index][4])
    for i=0, num_leds-1 do
        if i ~= light_list[list_index][1] then
          serialLED:set_RGB(chan, i, 0, 0, 0)
        end
    end
    list_index = list_index + 1
    if list_index > #light_list then
        list_index = 1
    end
    serialLED:send(chan)
    serialLED:send(chan)
    return update_LEDs, 2000
end

return update_LEDs, 2000

