% Updated: 09/26/2016 - Added rotMat_Explicit to reduce runtime
%          10/10/2016 - Added geometric center for base and link1
%          10/24/2016 - Added a solution to the IKP

classdef RobotArm_N1 < handle
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
        
        % Generic CAD data
        base
        link1
        link2
        link3
        link4
        link5
        link6
        
        % Manipulated CAD data
        base_M
        link1_M
        link2_M
        link3_M
        link4_M
        link5_M
        link6_M
        
        % Geometric center postion
        geometric_center
        geometric_center_M
        
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
        function this = RobotArm_N1(pose, edgecolor, quality)
            b = 166; % Height of base
            l1 = 124; % Distance from top facet of base to axis of link 1.
            l2 = 270; % Length between axes of link 2.
            L3a = 70; % Shortest distance from top axis of link 2 to axis of link 3. (possibly 69.96mm).
            L3b = 149.6; % Horizontal distance (in home position) from top axis of link 2 mutual face of link 3 with link 4.
            l4 = 152.4; % Distance from mutual face of link 3 and link 4 to axis of links 4-5.
            l5 = 72-13/2;%59; % Distance from axis of link 5 to its facet.
            l_EE = 60+13/2; % Distance from facet of link 5 to EE point
            
            this.L = [l1 l2 L3a L3b l4 l5];
            this.L_EE = l_EE;
            this.pose = pose;
            this.baseH = b;
            this.T_pose = [cosd(this.pose(3)) -sind(this.pose(3)) 0 this.pose(1)
                sind(this.pose(3)) cosd(this.pose(3)) 0 this.pose(2)
                0 0 1 0
                0 0 0 1];
            %Dir = 'C:\Users\Avishai\Documents\UIUC\Elastic rod\Motion planning\CAD\';
            %Dir = 'C:\Users\Avishai\Dropbox\UIUC\elasticrod\Motionplanning\CAD\'; % Windows
            Dir = '/home/avishai/Dropbox/UIUC/elasticrod/Motionplanning/CAD/'; % Ubuntu
            
            
            switch quality
                case 'coarse' % Take coarse mesh CAD (stl) files for faster performence
                    base = stlread([Dir 'base_r.STL']); base.facecolor = [0 0.4 0]; base.edgecolor = edgecolor; base.vertices = base.vertices;
                    link1 = stlread([Dir 'Link1_r.STL']); link1.facecolor = [1 0 0]; link1.edgecolor = edgecolor; link1.vertices = link1.vertices;
                    link2 = stlread([Dir 'Link2_r.STL']); link2.facecolor = [.2 .2 0]; link2.edgecolor = edgecolor; link2.vertices = link2.vertices;
                    link3 = stlread([Dir 'Link3_r.STL']); link3.facecolor = [.3 .0 .3]; link3.edgecolor = edgecolor; link3.vertices = link3.vertices;
                    link4 = stlread([Dir 'Link4_r.STL']); link4.facecolor = [.4 0 .9]; link4.edgecolor = edgecolor; link4.vertices = link4.vertices;
                    link5 = stlread([Dir 'Link5_r.STL']); link5.facecolor = [.5 .5 .5]; link5.edgecolor = edgecolor; link5.vertices = link5.vertices;
                    link6 = stlread([Dir 'EE_r.STL']); link6.facecolor = [1 1 0]; link6.edgecolor = edgecolor; link6.vertices = link6.vertices;
                case 'fine' % Take finer mesh CAD (stl) files for better presentation but slow simulation
                    base = stlread([Dir 'base.stl']); base.facecolor = [0 0.4 0]; base.edgecolor = edgecolor; base.vertices = base.vertices;
                    link1 = stlread([Dir 'link1.stl']); link1.facecolor = [1 0 0]; link1.edgecolor = edgecolor; link1.vertices = link1.vertices;
                    link2 = stlread([Dir 'link2.stl']); link2.facecolor = [.2 .2 0]; link2.edgecolor = edgecolor; link2.vertices = link2.vertices;
                    link3 = stlread([Dir 'link3.stl']); link3.facecolor = [.3 .0 .3]; link3.edgecolor = edgecolor; link3.vertices = link3.vertices;
                    link4 = stlread([Dir 'link4.stl']); link4.facecolor = [.4 0 .9]; link4.edgecolor = edgecolor; link4.vertices = link4.vertices;
                    link5 = stlread([Dir 'link5.stl']); link5.facecolor = [.5 .5 .5]; link5.edgecolor = edgecolor; link5.vertices = link5.vertices;
                    link6 = stlread([Dir 'EE_r.stl']); link6.facecolor = [1 1 0]; link6.edgecolor = edgecolor; link6.vertices = link6.vertices;
            end
            
            this.base = base;
            this.link1 = link1;
            this.link2 = link2;
            this.link3 = link3;
            this.link4 = link4;
            this.link5 = link5;
            this.link6 = link6;
            
            this.geometric_center = [0 0 80 1;
                0 0 230 1]';
            
            this.movePartsToOrigin();
            this.FK([0 0 0 0 0 0],1);
            
            % Joint limits - Real
            this.q1minmax(1) = deg2rad(165);
            this.q2minmax(1) = deg2rad(110);
            this.q3min(1) = deg2rad(-110); this.q3max(1) = deg2rad(70);
            this.q4minmax(1) = deg2rad(160);
            this.q5minmax(1) = deg2rad(120);
            this.q6minmax(1) = deg2rad(400);
            
            % Joint limits - with safety factor
            sf = 0.1;
            this.q1minmax(2) = (1-sf)*this.q1minmax(1);
            this.q2minmax(2) = (1-sf)*this.q2minmax(1);
            this.q3min(2) = (1-sf)*this.q3min(1); this.q3max(2) = (1-sf)*this.q3max(1);
            this.q4minmax(2) = (1-sf)*this.q4minmax(1);
            this.q5minmax(2) = (1-sf)*this.q5minmax(1);
            this.q6minmax(2) = (1-sf)*this.q6minmax(1);
            
            % Joint limits - relaxed
            sf = 0.1;
            this.q1minmax(3) = (1+sf)*this.q1minmax(1);
            this.q2minmax(3) = (1+sf)*this.q2minmax(1);
            this.q3min(3) = (1+sf)*this.q3min(1); this.q3max(3) = (1+sf)*this.q3max(1);
            this.q4minmax(3) = (1+sf)*this.q4minmax(1);
            this.q5minmax(3) = (1+sf)*this.q5minmax(1);
            this.q6minmax(3) = (1+sf)*this.q6minmax(1);
            
        end
        %% movePartsToOrigin
        function this = movePartsToOrigin(this)
            
            b = this.baseH;
            l1 = this.L(1);
            l2 = this.L(2);
            L3a = this.L(3);
            L3b = this.L(4);
            l4 = this.L(5);
            l5 = this.L(6);
            
            link1 = this.link1;
            link2 = this.link2;
            link3 = this.link3;
            link4 = this.link4;
            link5 = this.link5;
            link6 = this.link6;
            
            % Moving link1 to align with frame 0
            T = [1    0    0   0
                0    1    0   0
                0    0    1   -l1-b
                0    0    0    1];
            link1_new = this.transformation(link1, T);
            
            % Moving link2 to align with frame 0
            T = [1    0    0   0
                0    1    0   0
                0    0    1   -l1-b-l2
                0    0    0    1];
            link2_new = this.transformation(link2, T);
            
            % Moving link3 to align with frame 0
            T = [1   0    0   0
                0   1    0   0
                0   0    1    -l1-b-l2
                0   0    0    1];
            link3_new = this.transformation(link3, T);
            
            % Moving link4 to align with frame 0
            T = [1    0    0   -149.6-152.4;
                0    1    0   0
                0    0    1   -l1-b-270-70
                0    0    0    1];
            link4_new = this.transformation(link4, T);
            
            % Moving link5 to align with frame 0
            T = [1    0    0   -149.6-152.4
                0    1    0   0
                0    0    1   -l1-b-270-70
                0    0    0    1];
            link5_new = this.transformation(link5, T);
            
            % Moving link6 to align with frame 0
            T = [1    0    0   -20
                0    1    0   -20
                0    0    1   0
                0    0    0    1];
            link6_new = this.transformation(link6, T);
            
            T = [0   0    1    0
                0    1    0    0
                -1    0    0    0
                0    0    0    1];
            link6_new = this.transformation(link6_new, T);
            
            T = [1   0    0    0
                0    0    -1    0
                0    1    0    0
                0    0    0    1];
            link6_new = this.transformation(link6_new, T);
            
            % Ploting each part at the origin:
            
            % figure(2), clf, axis equal, view(3), xlabel('x'), ylabel('y'), zlabel('z'), grid on, hold on;
            % patch(this.base);
            % patch(link1_new);
            % patch(link2_new);
            % patch(link3_new);
            % patch(link4_new);
            % patch(link5_new);
            % patch(link6_new);
            % patch(end_effector_new);
            
            this.link1 = link1_new;
            this.link2 = link2_new;
            this.link3 = link3_new;
            this.link4 = link4_new;
            this.link5 = link5_new;
            this.link6 = link6_new;
        end
        %% transformation
        %Function takes the model_in and transitions it with the transformation matrix T.
        
        function model_out = transformation(this, model_in, T)
            
            model_out = model_in;
            for i = 1 : length(model_in.vertices(:, 1))
                
                model_out.vertices(i, :) = T(1 : 3, 1 : 3) * model_in.vertices(i,:)' + T(1 : 3, 4);
                
            end
            
        end
        %% FK
        function [T,RC] = FK(this, theta, ManipulateCAD)
            % This function returns a robot CADs configuration with \theta angles.
            % It also changes the *_M links in the RobotArm object.
            
            
            
            if ManipulateCAD
                [T1_0, T2_0, T3_0, T4_0, T5_0, T6_0] = this.rotMat(theta); %
                
                this.base_M = this.base;
                this.link1_M = this.transformation(this.link1, T1_0);
                this.link2_M = this.transformation(this.link2, T2_0);
                this.link3_M = this.transformation(this.link3, T3_0);
                this.link4_M = this.transformation(this.link4, T4_0);
                this.link5_M = this.transformation(this.link5, T5_0);
                this.link6_M = this.transformation(this.link6, T6_0);
                % end_effector_cur = transformation(end_effector_new, T6_0);
                
                % Move to pose
                this.base_M = this.transformation(this.base_M, this.T_pose);
                this.link1_M = this.transformation(this.link1_M, this.T_pose);
                this.link2_M = this.transformation(this.link2_M, this.T_pose);
                this.link3_M = this.transformation(this.link3_M, this.T_pose);
                this.link4_M = this.transformation(this.link4_M, this.T_pose);
                this.link5_M = this.transformation(this.link5_M, this.T_pose);
                this.link6_M = this.transformation(this.link6_M, this.T_pose);
                
                if nargout > 1
                    % Return configured CADs:
                    RC.base = this.base_M;
                    RC.link1 = this.link1_M;
                    RC.link2 = this.link2_M;
                    RC.link3 = this.link3_M;
                    RC.link4 = this.link4_M;
                    RC.link5 = this.link5_M;
                    RC.link6 = this.link6_M;
                end
            else
                T6_0 = this.rotMat_Explicit(theta);
            end
            
            % Manipulate geometric center
            this.geometric_center_M = this.T_pose*this.geometric_center; %T6_0*
            
            % EE position and orientation
            this.xyz = this.T_pose*T6_0*[this.L_EE; 0; 0; 1];
            this.xyz = this.xyz(1:3);
            this.T = this.T_pose*T6_0*[1 0 0 this.L_EE; 0 1 0 0; 0 0 1 0; 0 0 0 1];
            T = this.T;

        end 
        %% rotMat_Explicit
        function T6_0 = rotMat_Explicit(this, q)
            b = this.baseH;
            l1 = this.L(1);
            l2 = this.L(2);
            l3a = this.L(3);
            l3b = this.L(4);
            l4 = this.L(5);
            l5 = this.L(6);
            
            T6_0 = [ cos(q(5))*(cos(q(1))*cos(q(2))*cos(q(3)) - cos(q(1))*sin(q(2))*sin(q(3))) - sin(q(5))*(sin(q(1))*sin(q(4)) + cos(q(4))*(cos(q(1))*cos(q(2))*sin(q(3)) + cos(q(1))*cos(q(3))*sin(q(2)))), sin(q(6))*(cos(q(5))*(sin(q(1))*sin(q(4)) + cos(q(4))*(cos(q(1))*cos(q(2))*sin(q(3)) + cos(q(1))*cos(q(3))*sin(q(2)))) + sin(q(5))*(cos(q(1))*cos(q(2))*cos(q(3)) - cos(q(1))*sin(q(2))*sin(q(3)))) - cos(q(6))*(cos(q(4))*sin(q(1)) - sin(q(4))*(cos(q(1))*cos(q(2))*sin(q(3)) + cos(q(1))*cos(q(3))*sin(q(2)))),   sin(q(6))*(cos(q(4))*sin(q(1)) - sin(q(4))*(cos(q(1))*cos(q(2))*sin(q(3)) + cos(q(1))*cos(q(3))*sin(q(2)))) + cos(q(6))*(cos(q(5))*(sin(q(1))*sin(q(4)) + cos(q(4))*(cos(q(1))*cos(q(2))*sin(q(3)) + cos(q(1))*cos(q(3))*sin(q(2)))) + sin(q(5))*(cos(q(1))*cos(q(2))*cos(q(3)) - cos(q(1))*sin(q(2))*sin(q(3)))), cos(q(2) + q(3))*cos(q(1))*(l4 + l3b) - l5*(sin(q(1))*sin(q(4))*sin(q(5)) - cos(q(2) + q(3))*cos(q(1))*cos(q(5)) + cos(q(1))*cos(q(2))*cos(q(4))*sin(q(3))*sin(q(5)) + cos(q(1))*cos(q(3))*cos(q(4))*sin(q(2))*sin(q(5))) + l3a*sin(q(2) + q(3))*cos(q(1)) + l2*cos(q(1))*sin(q(2));
                sin(q(5))*(cos(q(1))*sin(q(4)) - cos(q(4))*(cos(q(2))*sin(q(1))*sin(q(3)) + cos(q(3))*sin(q(1))*sin(q(2)))) - cos(q(5))*(sin(q(1))*sin(q(2))*sin(q(3)) - cos(q(2))*cos(q(3))*sin(q(1))), cos(q(6))*(cos(q(1))*cos(q(4)) + sin(q(4))*(cos(q(2))*sin(q(1))*sin(q(3)) + cos(q(3))*sin(q(1))*sin(q(2)))) - sin(q(6))*(cos(q(5))*(cos(q(1))*sin(q(4)) - cos(q(4))*(cos(q(2))*sin(q(1))*sin(q(3)) + cos(q(3))*sin(q(1))*sin(q(2)))) + sin(q(5))*(sin(q(1))*sin(q(2))*sin(q(3)) - cos(q(2))*cos(q(3))*sin(q(1)))), - sin(q(6))*(cos(q(1))*cos(q(4)) + sin(q(4))*(cos(q(2))*sin(q(1))*sin(q(3)) + cos(q(3))*sin(q(1))*sin(q(2)))) - cos(q(6))*(cos(q(5))*(cos(q(1))*sin(q(4)) - cos(q(4))*(cos(q(2))*sin(q(1))*sin(q(3)) + cos(q(3))*sin(q(1))*sin(q(2)))) + sin(q(5))*(sin(q(1))*sin(q(2))*sin(q(3)) - cos(q(2))*cos(q(3))*sin(q(1)))), l5*(cos(q(2) + q(3))*cos(q(5))*sin(q(1)) + cos(q(1))*sin(q(4))*sin(q(5)) - cos(q(2))*cos(q(4))*sin(q(1))*sin(q(3))*sin(q(5)) - cos(q(3))*cos(q(4))*sin(q(1))*sin(q(2))*sin(q(5))) + cos(q(2) + q(3))*sin(q(1))*(l4 + l3b) + l3a*sin(q(2) + q(3))*sin(q(1)) + l2*sin(q(1))*sin(q(2));
                - sin(q(2) + q(3))*cos(q(5)) - cos(q(2) + q(3))*cos(q(4))*sin(q(5)),                                                                                                                                                              cos(q(2) + q(3))*cos(q(6))*sin(q(4)) - sin(q(6))*(sin(q(2) + q(3))*sin(q(5)) - cos(q(2) + q(3))*cos(q(4))*cos(q(5))),                                                                                                                                                              - cos(q(6))*(sin(q(2) + q(3))*sin(q(5)) - cos(q(2) + q(3))*cos(q(4))*cos(q(5))) - cos(q(2) + q(3))*sin(q(4))*sin(q(6)),                                                                      b + l1 - sin(q(2) + q(3))*(l4 + l3b) + l3a*cos(q(2) + q(3)) + l2*cos(q(2)) - l5*((cos(q(2) + q(3))*sin(q(4) + q(5)))/2 - (sin(q(4) - q(5))*cos(q(2) + q(3)))/2 + sin(q(2) + q(3))*cos(q(5)));
                0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                           0,                                                                                                                                                                                                                                 1];
        end
        %% rotMat
        function [T1_0, T2_0, T3_0, T4_0, T5_0, T6_0] = rotMat(this, q)
            
            b = this.baseH;
            l1 = this.L(1);
            l2 = this.L(2);
            L3a = this.L(3);
            L3b = this.L(4);
            l4 = this.L(5);
            l5 = this.L(6);
            
            T1_0 = this.Tt([0;0;l1+b]) * this.Tz(q(1));
            
            T2_0 = T1_0 * this.Ty(q(2)) * this.Tt([0; 0; l2]);
            
            T3_0 = T2_0 * this.Ty(q(3)) * this.Tt([0; 0; 0]);
            
            T4_0 = T3_0 * this.Tt([0; 0; L3a]) * this.Tx(q(4)) * this.Tt([L3b + l4; 0; 0]);
            
            T5_0 = T4_0 * this.Ty(q(5));
            
            T6_0 = T5_0 * this.Tx(q(6)) * this.Tt([l5; 0; 0]);
            
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
        %% Robot2Fig
        function Robot2Fig(this, figNum, RC)
            % This function plots the robot in the RC conf if available
            % If no, plots the current state of RobotArm from *_M.
            
            figure(figNum)
            
            if nargin > 2
                patch(RC.base);
                patch(RC.link1);
                patch(RC.link2);
                patch(RC.link3);
                patch(RC.link4);
                patch(RC.link5);
                patch(RC.link6);
            else
                patch(this.base_M);
                patch(this.link1_M);
                patch(this.link2_M);
                patch(this.link3_M);
                patch(this.link4_M);
                patch(this.link5_M);
                patch(this.link6_M);
            end
            
            axis equal
            xlabel('x');
            ylabel('y');
            zlabel('z');
            
%             hold on
%             plot3(this.geometric_center_M(1,1:2),this.geometric_center_M(2,1:2),this.geometric_center_M(3,1:2),'*r');
%             hold off
            
            grid
            view(3)%0,0)
            
        end
        %% IKP
        function [q, fail] = IKP(this, T, LimitsMode, solution_num)
            % Inverse kinematics calculation for the ABB IRB 120 robot
            % T - is the desired position and orientation matrix of the gripper in the
            % world CF
            % LimitsMode - 1 for real limits and 2 for bias (safety factor)
            % limits, 3- relaxed limits
            % fail - 1 if failed to find a solution, 0 - found a solution.
            
            if nargin < 4
                solution_num = 1;
            end
%             q1_add =  [ 0  0  0  0 -pi -pi -pi -pi];
%             q2_sign = [-1 -1  1  1  -1  -1   1   1];
%             sign456 = [ 1 -1  1 -1   1  -1   1  -1];
            q1_add  = [ 0  0  0  0 pi pi pi pi -pi -pi -pi -pi];
            q2_sign = [-1 -1 -1 -1  1  1  1  1   1   1   1   1];
            q3_sign = [ 1 -1  1 -1  1 -1  1 -1   1  -1   1  -1];
            sign456 = [ 1 -1 -1  1 -1  1  1 -1  -1   1   1  -1];

            
            q = zeros(6,1);
            fail = 1;
            
            b = this.baseH;
            L1 = this.L(1);
            L2 = this.L(2);
            L3a = this.L(3);
            L3b = this.L(4);
            L4 = this.L(5);
            L5 = this.L(6);
            L_EE = this.L_EE;
            
            L34 = (L3a^2 + (L3b+L4)^2)^0.5; % Distance from joint 23 axis to joint 45 axis.
            alpha = atan2(L3b+L4, L3a); % Angle in the base of the triangle of link 3.
            
            %% %%
            
            if any(size(T)==1)
                
                % Gripper position
                x = T(1);
                y = T(2);
                z = T(3);
                tr = T(4); % roll
                tp = T(5); % pitch relative to the world z axis. That is, zero pitch is the ee pointing up
                ty = T(6); % yaw
                
                % Gripper orientation matrix
                cy = cos(ty);
                cp = cos(tp);
                cr = cos(tr);
                sy = sin(ty);
                sp = sin(tp);
                sr = sin(tr);
                R = this.T_pose(1:3,1:3)*[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr;
                    sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr;
                    -sp, cp*sr, cp*cr];
            else
                % Gripper position
                T = this.T_pose\T;
                x = T(1,4);
                y = T(2,4);
                z = T(3,4);
                
                % Gripper orientation matrix
                R = T(1:3,1:3);
            end
            
            x6 = R(:,1); % Gripper x-axis
            
            %% %%
            % q1
            p5 = [x;y;z] - x6*(L5+L_EE);
            q1 = atan2(p5(2),p5(1))+q1_add(solution_num); % If sign2 (on q2) equals 1, add -pi to q1.
            if abs(q1) > this.q1minmax(LimitsMode)
                return;
            end
            
            % q3
            k2 = (p5(1)^2 + p5(2)^2); % = k^2
            D2 = k2 + (p5(3)-(L1+b))^2; % = D^2
            cosphi = (D2-L2^2-L34^2)/(2*L2*L34);
            sinphi = q3_sign(solution_num)*(1-cosphi^2)^0.5;
            if imag(sinphi)
                return;
            end
%             phi = atan2(sinphi,cosphi);
%             q3 = -(phi - pi + alpha);
            q3 = -(atan2(sinphi,cosphi)+alpha);
            if q3 < this.q3min(LimitsMode) || q3 > this.q3max(LimitsMode)
                return;
            end
            
            % q2
            sinalpha1 = -L34*(sinphi)/(D2^0.5);
            alpha1 = atan2(-q2_sign(solution_num)*sinalpha1,(1-sinalpha1^2)^0.5);
            alpha2 = atan2(p5(3)-L1-b,k2^0.5);
            q2 = q2_sign(solution_num)*(alpha1 + alpha2 - pi/2);
            % if q2 > pi; q2 = q2 - 2*pi; end;
            if abs(q2) > this.q2minmax(LimitsMode)
                return;
            end
            
            
            %% %%%
            
            % Solved according to Spong's "Robot dynamics and control" page 91,96.
            
            
            c23 = cos(q2+q3); s23 = sin(q2+q3);
            c1 = cos(q1); s1 = sin(q1);
            
            % q5
            % It is possible to use the cross to find the sign of q5 and by
            % that, merge between two IK solutions.
%             x4 = [c23*c1; c23*s1; -s23];
%             S = dot(x4,x6);
%             sgn = sign(cross(x4,x6)); sgn = sgn(2);
%             q5 = sgn*atan2(sign456(solution_num)*(1-S^2)^0.5, S);
            S = R(1,1)*c23*c1 - R(3,1)*s23 + R(2,1)*c23*s1;
            q5 = atan2(sign456(solution_num)*(1-S^2)^0.5, S);
            q5(abs(q5)<1e-4) = 0;
            if abs(q5) > this.q5minmax(LimitsMode)
                return;
            end
            
            if q5
                % q4
                sinq4 = R(2,1)*c1 - R(1,1)*s1;
                cosq4 = R(3,1)*c23 + R(1,1)*s23*c1 + R(2,1)*s23*s1;
                q4 = atan2(sign456(solution_num)*sinq4, -sign456(solution_num)*cosq4);
%                 q4 = atan2(sgn*sinq4, -sgn*cosq4);
                
                % q6
                sinq6 = R(1,2)*c23*c1 - R(3,2)*s23 + R(2,2)*c23*s1;
                cosq6 = R(1,3)*c23*c1 - R(3,3)*s23 + R(2,3)*c23*s1;
                q6 = atan2(sign456(solution_num)*sinq6, sign456(solution_num)*cosq6);
%                 q6 = atan2(sgn*sinq6, sgn*cosq6);
            else
                % q4 + q6 = cons.
                % We choose to set q4=0 and then decide q6.
                q4 = 0;
                q6 = atan2(R(3,2), R(3,3)); % or q6=atan2(R(3,2), R(2,2)) or q6=atan2(-R(2,3), R(3,3));
            end
            if abs(q6) >= this.q6minmax(LimitsMode) || abs(q4) >= this.q4minmax(LimitsMode)
                return;
            end
            
            fail = 0;
            q = [q1; q2; q3; q4; q5; q6];
            q(abs(q)<1e-4) = 0;
%             rad2deg(q)% phi])
                        
        end
        %% fix_joints_4_6
        function Q2 = fix_joints_4_6(this, Q2)
            q4 = Q2(4,:);
            q6 = Q2(6,:);
            n = size(q4,2);
            j = 2;
            while j < n
                i = j;
                while i < n && sign(q4(i)*q4(i-1))>0 
                    i = i + 1;
                end
                if i==n; break; end
                if abs(q4(i)-q4(i-1))<0.2
                    j = i + 1;
                    continue;
                end
                
                k = i + 1;
                while k < n && sign(q4(k)*q4(k-1))>0 
                    k = k + 1;
                end
                if (sign(q4(k)*q4(k-1))<1) && abs(q4(i)-q4(i-1))>0.2
                    q4(i:k-1) = q4(i:k-1) - pi;
                else if k==n
                        q4(i:n) = q4(i:n) - pi;
                        break;
                    end
                end
                j = k;
            end
            Q2(4,:) = q4;
            
%             j = 2;
%             while j < n
%                 i = j;
%                 while i < n && sign(q6(i)*q6(i-1))>0 
%                     i = i + 1;
%                 end
%                 if i==n; break; end
%                 if abs(q6(i)-q6(i-1))<0.2
%                     j = i + 1;
%                     continue;
%                 end
%                 
%                 k = i + 1;
%                 while k < n && sign(q6(k)*q6(k-1))>0 
%                     k = k + 1;
%                 end
%                 if (sign(q6(k)*q6(k-1))<1) && abs(q6(i)-q6(i-1))>0.2
%                     q6(i:k-1) = q6(i:k-1) - pi;
%                 else if k==n
%                         q6(i:n) = q6(i:n) - pi;
%                         break;
%                     end
%                 end
%                 j = k;
%             end
%             Q2(6,:) = q6;
        end
    end

end

