% Updated:

classdef BaxterKinematics_B1 < handle
    properties
        pose % desired position and orientation of the robot on the x-y plane.
        edgecolor % incluFde edgecolor 'c' or no 'none'.
        quality % use 'fine' mesh or 'coarse'.
        L % Links lengths
        L_EE % EE lenght
        baseH % Base height
        T_pose % Robot position and orientation matrix
        xyz % Current position of EE
        T % Current transformation matrix of EE
        
        b % base
        l1
        l2
        l3
        l4
        l5
        l6
        l7
        lEE
        
        
        % Joint limits
        q1minmax
        q2minmax
        q3max
        q3min
        q4minmax
        q5minmax
        q6minmax
    end
    
    methods
        %% RobotArm
        function this = BaxterKinematics_B1()
            this.b = 1000*[0;0;0];
            this.l1 = 1000*[0.024645+0.055695-0.02; -0.25; 0.118588+0.011038];
            this.l2 = 1000*[0.073; 0; 0.245];
            this.l3 = 1000*[0.102; 0; 0];
            this.l4 = 1000*[0.069; 0; 0.26242-0.015];
            this.l5 = 1000*[0.10; 0; 0];
            this.l6 = 1000*[0.01; 0; 0.2707];
            this.l7 = 1000*[0.16; 0; 0];
            this.lEE = 1000*[0; 0; 0.05]+[0;0;0.1*1000]; % Extension by 0.1 to be at the middle of the gripper
            
            % Joint limits - Real
            this.q1minmax(1) = deg2rad(165);
            this.q2minmax(1) = deg2rad(110);
            this.q3min(1) = deg2rad(-110); this.q3max(1) = deg2rad(70);
            this.q4minmax(1) = deg2rad(160);
            this.q5minmax(1) = deg2rad(120);
            this.q6minmax(1) = deg2rad(400);
            
        end
        
        %% FK
        function T = FK(this, theta, robot_number)
            theta1 = rad2deg(theta);
            
            L1 = this.l1;
            if robot_number == 2
                L1(2) = -L1(2);
            end
            
            T01 = this.Tt(this.b) * this.Tt(L1) * this.Tz(theta(1));
            T02 = T01*this.Tt(this.l2)*this.Ty(theta(2))*this.Tx(-pi/2);
            T03 = T02*this.Tt(this.l3)*this.Tx(theta(3)+pi/2)*this.Ty(pi/2);
            T04 = T03*this.Tt(this.l4)*this.Ty(theta(4))*this.Tx(pi/2);
            T05 = T04*this.Tt(this.l5)*this.Tx(theta(5)-pi/2)*this.Ty(pi/2);
            T06 = T05*this.Tt(this.l6)*this.Ty(theta(6)-pi/2)*this.Tx(pi/2);
            T07 = T06*this.Tt(this.l7)*this.Tx(theta(7)+pi/2)*this.Ty(pi/2);
            T0EE = T07 * this.Tt(this.lEE); 

            T = T0EE;
            
        end
        
        %% Tz Tx Ty
        function T = Tz(this, x)
            
            T = [cos(x) -sin(x) 0 0; sin(x) cos(x) 0 0; 0 0 1 0; 0 0 0 1];
            
        end
        
        function T = Ty(this, x)
            
            T = [cos(x) 0 sin(x) 0; 0 1 0 0; -sin(x) 0 cos(x) 0; 0 0 0 1];
            
        end
        
        function T = Tx(this, x)
            
            T = [1 0 0 0; 0 cos(x) -sin(x) 0; 0 sin(x) cos(x) 0; 0 0 0 1];
            
        end
        
        function T = Tt(this, v)
            
            T = [[eye(3) v]; 0 0 0 1];
            
        end
        
        %%
        function print2file(this, thetas)
            fileID = fopen('../../paths/path.txt','w');
            fprintf(fileID,'1\n');
            for i = 1:size(thetas)
                fprintf(fileID,'%f ',thetas(i));
            end
            fclose(fileID);
            
        end
        
        
        
    end
    
end

