module(... or '', package.seeall)

-- Get Platform for package path
cwd = '.';
local platform = os.getenv('PLATFORM') or '';
if (string.find(platform,'webots')) then cwd = cwd .. '/Player';
end

-- Get Computer for Lib suffix
local computer = os.getenv('COMPUTER') or '';
if (string.find(computer, 'Darwin')) then
  -- MacOS X uses .dylib:
  package.cpath = cwd .. '/Lib/?.dylib;' .. package.cpath;
else
  package.cpath = cwd .. '/Lib/?.so;' .. package.cpath;
end

package.path = cwd .. '/?.lua;' .. package.path;
package.path = cwd .. '/Util/?.lua;' .. package.path;
package.path = cwd .. '/Config/?.lua;' .. package.path;
package.path = cwd .. '/Lib/?.lua;' .. package.path;
package.path = cwd .. '/Dev/?.lua;' .. package.path;
package.path = cwd .. '/Motion/?.lua;' .. package.path;
package.path = cwd .. '/Motion/keyframes/?.lua;' .. package.path;
package.path = cwd .. '/Motion/Walk/?.lua;' .. package.path;
package.path = cwd .. '/Vision/?.lua;' .. package.path;
package.path = cwd .. '/World/?.lua;' .. package.path;

require('unix')
require('Config')
require('shm')
require('vector')
require('mcm')
require('Speak')
require('getch')
require('Body')
require('Motion')
require('dive')
require('grip')
require('socket')

-------------- UDP COMMUNICATION FOR BODY KINEMATIC ----------
-- local socket_body = require "socket"
local udp_body = socket.udp()
udp_body:settimeout(0)
udp_body:setsockname('*', 8080)
local data_body, msg_or_ip_body, port_or_nil_body
-------------------------------------------------------

--------------- UDP COMMUNICATION FOR HEAD MOVEMENT --------------
-- local socket_head = require "socket"
local udp_head = socket.udp()
udp_head:settimeout(0)
udp_head:setsockname('*', 8081)
local data_head, msg_or_ip_head, port_or_nil_head
---------------------------------------------------

--------------- UDP COMMUNICATION FOR FOOT COMP --------------
-- local socket_comp = require "socket"
local udp_comp = socket.udp()
udp_comp:settimeout(0)
udp_comp:setsockname('*', 8082)
local data_comp, msg_or_ip_comp, port_or_nil_comp
---------------------------------------------------

--------------- UDP COMMUNICATION FOR BUTTON --------------
local socket_button = require "socket"
local udp_button = socket.udp()
udp_button:settimeout(0)
udp_button:setpeername("127.0.0.1", 8083)
---------------------------------------------------

Motion.entry();
darwin = false;
webots = false;

-- Enable OP specific 
if(Config.platform.name == 'OP') then
  darwin = true;
  --SJ: OP specific initialization posing (to prevent twisting)
--  Body.set_body_hardness(0.3);
--  Body.set_actuator_command(Config.stance.initangle)
  footXComp = Config.walk.footXComp;
end

--TODO: enable new nao specific
newnao = false; --Turn this on for new naos (run main code outside naoqi)
newnao = true;

getch.enableblock(1);
unix.usleep(1E6*1.0);
Body.set_body_hardness(0);

--This is robot specific 
webots = false;
init = false;
calibrating = false;
ready = false;
backward = false;
if( webots or darwin) then
  ready = true;
end

initToggle = true;
targetvel=vector.zeros(3);
button_pressed = {0,0};

function string:split(inSplitPattern, outResults)
   if not outResults then
      outResults = { }
   end
   local theStart = 1
   local theSplitStart, theSplitEnd = string.find( self, inSplitPattern, theStart )
   while theSplitStart do
      table.insert( outResults, string.sub( self, theStart, theSplitStart-1 ) )
      theStart = theSplitEnd + 1
      theSplitStart, theSplitEnd = string.find( self, inSplitPattern, theStart )
   end
   table.insert( outResults, string.sub( self, theStart ) )
   return outResults
end

function openGripper()
    Body.set_aux_hardness(0.5);
    angle = math.pi/180*vector.new({60, 60})
    Body.set_aux_command(angle);
end

function closeGripper()
  Body.set_aux_hardness(0.5);
  angle = math.pi/180*vector.new({0, 0})
  Body.set_aux_command(angle);
end

os.execute("screen -d player");
function process_keyinput()  
  --hanjaya
  data_body, msg_or_ip_body, port_or_nil_body = udp_body:receivefrom(20)
  if data_body ~= nil then
    -- print(data_body)
    if data_body then
      --print(data_body)
      local byte=string.byte(data_body);
      
      if (data_body=="stand") then	
        Motion.event("standup");
        closeGripper();
      
      elseif (data_body=="start") then	
        Motion.event("start");
        walk.start();
      
      elseif (data_body=="stop") then	
        if walk.active then walk.stop(); end
     
      elseif (data_body=="sit") then	
        Motion.event("sit");
           
      else 
	      local body_data = data_body:split(" ")

        -- Walking velocity
        if (body_data[1]=="walk") then
          if walk.active then 
            walk.set_velocity(tonumber(body_data[2]),tonumber(body_data[3]),tonumber(body_data[4]));
            if tonumber(body_data[2]) < 0 then
              backward = true;
            else
              backward = false;
            end
          else
            walk.start();
          end
		      -- print("X:",body_data[2], "Y:",body_data[3], "A:",body_data[4])

        -- Kick
        elseif (body_data[1]=="action") then
		      --print("Action:",body_data[2])
		      if(tonumber(body_data[2]) == 1) then
		        kick.set_kick("kickForwardLeft");		        
		      elseif(tonumber(body_data[2]) == 2) then
		        kick.set_kick("kickForwardRight");		        
          end		
          Motion.event("kick");

        -- Gripper
        elseif (body_data[1]=="grip") then
		      --print(body_data[2])
		      if(tonumber(body_data[2]) == 1) then
		        closeGripper();
		      else 
		        openGripper();
		      end
	      end
      end

      -- for walking in the begining
      --walk.set_velocity(unpack(targetvel));
    end
  end
  
  -- Head Movement
  data_head, msg_or_ip_head, port_or_nil_head = udp_head:receivefrom()
  if data_head then
    -- print(data_head)
    local head_angle = data_head:split(" ")
    Body.set_head_hardness(1);
    Body.set_head_command({tonumber(head_angle[2]),tonumber(head_angle[3])});

  elseif msg_or_ip_head ~= 'timeout' then
  end

  -- Foot X Comp
  data_comp, msg_or_ip_comp, port_or_nil_comp = udp_comp:receivefrom()
  if data_comp then
    print(data_comp)
    local footXComp = data_comp:split(" ")[2] 
    mcm.set_walk_footXComp(tonumber(footXComp));

  elseif msg_or_ip_comp ~= 'timeout' then
  end
end

function balance_xcomp()
  local imuAngle=vector.new(Body.get_sensor_imuAngle())*180/math.pi;

  if backward then
    -- print(string.format("Angle: %.1f %.1f %.1f ",unpack(imuAngle)));
    -- print(string.format("Pitch: %.1f",imuAngle[2]));
    error_p   = imuAngle[2] - 15.0;
    p_xcomp   = error_p * 0.0005;
    -- p_xcomp = p_xcomp * -1;
    ap_xcomp = p_xcomp + footXComp;
    ap_xcomp = tonumber(string.format("%.4f", ap_xcomp))

    --  limiter
    if ap_xcomp >= -0.002 then
      ap_xcomp = -0.002
    elseif ap_xcomp <= -0.015 then
      ap_xcomp = -0.015
    end
    -- print(string.format("Balance x_comp: %.4f", ap_xcomp));
    mcm.set_walk_footXComp(ap_xcomp);
  end
end

-- main loop
count = 0;
lcount = 0;
tUpdate = unix.time();

function update()
  count = count + 1;
  if (not init)  then
    if (calibrating) then
      if (Body.calibrate(count)) then
        Speak.talk('Calibration done');
        calibrating = false;
        ready = true;
      end
    elseif (ready) then
      init = true;
    else
      if (count % 20 == 0) then
-- start calibrating w/o waiting
--        if (Body.get_change_state() == 1) then
          Speak.talk('Calibrating');
          calibrating = true;
--        end
      end
      -- toggle state indicator
      if (count % 100 == 0) then
        initToggle = not initToggle;
        if (initToggle) then
          Body.set_indicator_state({1,1,1}); 
        else
          Body.set_indicator_state({0,0,0});
        end
      end
    end
  else

    -- update state machines 
    process_keyinput();
    Motion.update();
    Body.update();
    balance_xcomp();
  end
  local dcount = 50;
  if (count % 50 == 0) then
--    print('fps: '..(50 / (unix.time() - tUpdate)));
    tUpdate = unix.time();
    -- update battery indicator
    Body.set_indicator_batteryLevel(Body.get_battery_level());
  end
  
  -- check if the last update completed without errors
  lcount = lcount + 1;
  if (count ~= lcount) then
    print('count: '..count)
    print('lcount: '..lcount)
    Speak.talk('missed cycle');
    lcount = count;
  end

  -- read button state
  if (Body.get_change_state() == 1) then button_pressed[1]=1;
  else                                   button_pressed[1]=0;
  end

  if (Body.get_change_role() == 1) then button_pressed[2]=1;
  else                                  button_pressed[2]=0;
  end

  -- print(vector.new(button_pressed))
  datagram = string.format("%d %d", unpack(button_pressed)) 
  udp_button:send(datagram)

  -- --Stop walking if button is pressed and the released
  -- if (Body.get_change_state() == 1) then
  --   button_pressed[1]=1;
  -- else
  --   if button_pressed[1]==1 then
  --     Motion.event("sit");
  --   end
  --   button_pressed[1]=0;
  -- end

  -- --stand up if button is pressed and the released
  -- if (Body.get_change_role() == 1) then
  --   button_pressed[2]=1;
  -- else
  --   if button_pressed[2]==1 then
  --     Motion.event("standup");
  --   end
  --   button_pressed[2]=0;
  -- end
  
end

-- if using Webots simulator just run update
if (webots) then
  while (true) do
    -- update motion process
    update();
    io.stdout:flush();
  end
end

--Now both nao and darwin runs this separately
if (darwin) or (newnao) then
  local tDelay = 0.005 * 1E6; -- Loop every 5ms
  while 1 do
    update();
    unix.usleep(tDelay);
  end
end
