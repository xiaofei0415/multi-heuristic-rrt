classdef myrrt <handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
         %MaxConnectionDistance - Maximum length between planned configurations
        %   The length of the motion is computed as the Euclidean distance
        %   between two node configurations. Difference between two joint
        %   positions for a revolute joint is calculated using angDiff(). If the
        %   connect heuristic is enabled, then this property is ignored at the
        %   joining stage during planning.
        %
        %   Default: 0.1
        MaxConnectionDistance = 0.4;
               %ValidationDistance - Distance resolution for validating motion between configurations
        %   The validation distance determines the number of interpolated nodes
        %   between two adjacent nodes of the tree that should be checked for
        %   validity.
        %
        %   Default: 0.01
        ValidationDistance = 0.2;
        
        Xsoln=[];%index of Tree ,column vector
        cbest=Inf; 
        change = 0;
        start=[];
        goal=[];
        Ta=[];
        Tb=[];
        num1 = 0;%inital path found
        n=2500;
        optimal_path=[];
        isInitialPathFound=0;
        ellipseNodesA = [];%index of Tree ,column vector
        ellipseNodesB = [];
        cmin=0;
        C=[];
        x_center=[];
        pre_cbest = Inf;
        pre_path=[];
        jointBounds=[];
        itered = 0;
        %StateValidator The instance of a ManipulatorStateValidator
        %   The StateValidator will validate a node or an edge in the
        %   SearchTree. Note that the node in the search tree corresponds to a
        %   single joint configuration, and edge is a motion that comprises
        %   of a series of joint configurations.
        StateValidator;
        ss;
    end  
   

    methods(Access=private)
        function cname = getClassName(~)
        %getClassName Returns the name of the class
            cname = "myrrt";
        end

        function updateValidationDistance(obj, val)
        %updateValidationDistance Updates the validation distance of the underlying StateValidator
            obj.StateValidator.ValidationDistance = val;
        end
    end
    
    
    
    methods
        
        function obj = myrrt(robot, environment)
        %MANIPULATORRRT Constructor
            obj.ss = robotics.manip.internal.ManipulatorStateSpace(robot);
            obj.StateValidator = ...
                robotics.manip.internal.ManipulatorStateValidator(...
                obj.ss, environment, 0.2);%obj.ValidationDistance is OK??
            [~, obj.jointBounds] = ...
                robotics.manip.internal.RigidBodyTreeUtils.getConfigurationInfo(robot);  
                
        end

        function interpolatedPath = interpolate(obj, path, varargin)
        %interpolate Interpolate states along path
        %   INTERPOLATEDPATH = interpolate(PLANNER, PATH) interpolates
        %   between each adjacent configuration in the based on the
        %   ValidationDistance property.
        %
        %   INTERPOLATEDPATH = interpolate(PLANNER, PATH, NUMINTERPOLATIONS)
        %   NUMINTERPOLATIONS is the number of linear interpolations between
        %   each adjacent configuration in the path.
        %
        %   PATH is R-by-N matrix where R is the number of configurations,
        %   and N corresponds to the dimension of the configuration of the
        %   robot. Each row in PATH corresponds to a configuration.

            narginchk(2, 3);
            if(nargin == 3)
                numInterpolations = varargin{1};
                interpolatedPath = ...
                    robotics.manip.internal.RRTUtils.interpolateByNumber(...
                        path, obj.StateValidator.StateSpace, numInterpolations);
            else
                interpolatedPath = ...
                    robotics.manip.internal.RRTUtils.interpolateByResolution(...
                        path, obj.StateValidator.StateSpace, obj.ValidationDistance);
            end
        end
        
        function newObj = copy(obj)
        %copy Creates a copy of the planner
            robot = obj.StateValidator.StateSpace.Robot;
            env = obj.StateValidator.Environment;
            newObj = myrrt(robot, env);
            
        end
        
%         
%         function obj = myrrt()
%             %UNTITLED Construct an instance of this class
%             %   Detailed explanation goes here
%             obj = plan(obj);
%         end
        
        function obj =  plan(obj,startConfig, goalConfig)
            if(~obj.StateValidator.isStateValid(startConfig))
                robotics.manip.internal.error('manipulatorplanning:InvalidStartConfiguration');
            end
            if(~obj.StateValidator.isStateValid(goalConfig))
                robotics.manip.internal.error('manipulatorplanning:InvalidGoalConfiguration');
            end
            
            obj.optimal_path=[];
            obj.isInitialPathFound=0;
            obj.ellipseNodesA = [];%index of Tree ,column vector
            obj.ellipseNodesB = [];
            obj.Xsoln=[];%index of Tree ,column vector
            obj.cbest=Inf; 
            obj.pre_cbest = Inf;
            obj.pre_path=[];
            obj.itered=0;
            
            obj.Ta=[startConfig 0 0];
            obj.Tb=[goalConfig 0 0];
            obj.start = startConfig;
            obj.goal = goalConfig;
            obj.cmin=norm(goalConfig-startConfig);
            obj.x_center=((startConfig+goalConfig)/2)';
            a_l=(goalConfig-startConfig)/obj.cmin;
            a_l = a_l';
            id_t=[1,0,0,0,0,0,0];
            M=a_l*id_t;
            [U,~,Vh] = svd(M);
            obj.C=(U*diag([1,1,1,1,1,1,det(U)*det(Vh')]))*(Vh);
            flag1 = 0;
            obj.num1=0;
            for i=1:obj.n
                tic;
                obj.itered = i;
                if obj.isInitialPathFound && mod(i-obj.num1,50)==1
                    [obj.cbest,obj.optimal_path] = obj.caculateShortestPath();
                    disp("---------------------------------------------------");
                
                    if  obj.pre_cbest>obj.cbest
                        obj.change  = obj.change+1;
                        disp(["newcbest-----------------------------;;;;;;;;;;;;;;;;;;;-",obj.cbest]);
                        [obj.ellipseNodesA, obj.ellipseNodesB] = obj.updateEllipseNodes();
                        if size(obj.ellipseNodesA,1) ==0
                            disp("bye");
                            break;
                        end
                        %disp("ifellip");disp(obj.ellipseNodesA');
                        obj.Xsoln= obj.updateXsoln();
                        
                        obj.pre_cbest = obj.cbest;
                        obj.pre_path = obj.optimal_path;
                        
                    end
                    if obj.n-i < 50
                        break;
                    end           
                    
                end
                
                randpoint = obj.mysample(obj.cbest);   
                
                sign2="hhh";
                [sign1, newpoint,index1, obj.Ta] = obj.extend(obj.Ta, obj.ellipseNodesA,0,randpoint);
                if sign1~="Trapped"
                    
                    [sign2, ~, index2,obj.Tb] = obj.extend(obj.Tb,obj.ellipseNodesB,0, newpoint);
%                     while ~obj.isInitialPathFound&&sign2 == "Advanced"
%                         [sign2, ~, index2,obj.Tb] = obj.extend(obj.Tb,obj.ellipseNodesB,index2, newpoint);
%                     end
                    
                end
                
                if obj.cbest<Inf&&sign1 ~="Trapped"
                    obj.ellipseNodesA(end+1,:)= index1;
                    %disp("Acbestellip");disp(index1);
                end
                if obj.cbest<Inf&& (sign2 =="Advanced"||sign2=="Reached")
                    obj.ellipseNodesB(end+1,:)= index2;
                    %disp("Bcbestellip");disp(index2);
                end
                if sign1 ~="Trapped" && sign2 == "Reached"
                    obj.Xsoln=obj.addNewSolution(index1, index2);
                    obj.isInitialPathFound = 1;
                    flag1 = flag1+1;
                    
                    if flag1 == 1
                        disp("initalfound");
                        obj.num1 = i;
                    end
                end
                
                %swap
                temp=obj.Ta;
                obj.Ta = obj.Tb;
                obj.Tb= temp;
                if obj.cbest<Inf
                    temp2 = obj.ellipseNodesA;
                    obj.ellipseNodesA = obj.ellipseNodesB;
                    obj.ellipseNodesB = temp2;
                end
%                 if i == 100 || i == 1000||i==300||i==600
%                     plot(obj);
%                 end
                disp(["iters:",i]);
                toc;
                
            end
            
        end
%         function plot(obj)
%             figure;
%             hold on;
%             grid on;
%             plot([0 0 100 100 0], [0 100 100 0 0],'c-','LineWidth', 1.5);%cyan
%             %fill([35 25 25 75 75 65 65 35 35], [35 35 65 65 35 35 55 55 35],'c');
%             for i = 1:size(obj.obs,1)
%                 tem=obj.obs{i,1};
%                 plot([tem(1,1),tem(1,1),tem(2,1),tem(2,1),tem(1,1)], [tem(1,2),tem(2,2),tem(2,2),tem(1,2),tem(1,2)],'c-','LineWidth', 1.5);%cyan
%             end
%             
%             if abs(obj.Ta(1,1)-obj.start(1,1))<0.01 && abs(obj.Ta(1,2)-obj.start(1,2))<0.01
%                 Xa = obj.Ta(:,1:3);
%                 Xb= obj.Tb(:,1:3);
%             else
%                 Xa= obj.Tb(:,1:3);
%                 Xb= obj.Ta(:,1:3);
%             end
%             
%             for i=1:size(obj.Ta,1)
%                 if i==1
%                     plot(Xa(1,1),Xa(1,2),'*r');
%                 else
%                     index = Xa(i,3);
%                     plot([Xa(i,1) Xa(index,1)],[Xa(i,2),Xa(index,2)], 'b-');
%                     plot(Xa(i,1),Xa(i,2), 'k.', 'MarkerSize', 8);
%                 end
%             end
%             
%             for i=1:size(Xb,1)
%                 if i==1
%                     plot(Xb(1,1),Xb(1,2),'*b');
%                 else
%                     index = Xb(i,3);
%                     plot([Xb(i,1) Xb(index,1)],[Xb(i,2),Xb(index,2)], 'r-');
%                     plot(Xb(i,1),Xb(i,2), 'k.', 'MarkerSize', 8);
%                 end
%             end
%             
%             drawnow;
%             
%             if obj.isInitialPathFound           
%                 fprintf("obj.cbest:  ");disp(obj.cbest);        
%                 
%                 x1=obj.x_center(1);
%                 y1=obj.x_center(2);
%                 a1=obj.cbest/2;
%                 v=obj.goal-obj.start;
%                 b1=sqrt(obj.cbest.^2-obj.cmin.^2)/2;
%                 sita=0:pi/100:2*pi;
%                 fi1=atan2(v(2),v(1));
%                 %plot ellipse
%                 plot(x1+a1*cos(fi1)*cos(sita)-b1*sin(fi1)*sin(sita),y1+a1*sin(fi1)*cos(sita)+b1*cos(fi1)*sin(sita),'--g');
%                 %plot optimal path
%                 for i = 1:size(obj.optimal_path,1)-1
%                     plot([obj.optimal_path(i,1),obj.optimal_path(i+1,1)],[obj.optimal_path(i,2),obj.optimal_path(i+1,2)],'y-','LineWidth', 1.5);
%                 end
%                 %plot all Xsoln point             
%                 for i = 1:size(obj.Xsoln,1)
%                     in = obj.Xsoln(i,1);
%                     plot(Xa(in,1),Xa(in,2),'og');
%                 end
%                 drawnow;
%             else
%                 disp("no inital path yet");          
%             end
%             hold off;
%         end
        
        function [ellipseA,ellipseB] = updateEllipseNodes(obj)
            %after first path found,update the ellipse nodes of tree a and
            %tree b ,where extend () works
            nn= size(obj.Ta,1);
            
            m = obj.Ta(:,1:7)-ones(nn,1)*obj.start;
            tem1 = sqrt(sum(m.^2, 2));
      
            m = obj.Ta(:,1:7)-ones(nn,1)*obj.goal;
            tem2 = sqrt(sum(m.^2, 2));
            tem1=tem1+tem2;
            ellipseA = find(tem1<obj.cbest);
            
            nn= size(obj.Tb,1);
            
            m = obj.Tb(:,1:7)-ones(nn,1)*obj.start;
            tem1 = sqrt(sum(m.^2, 2));
            
            m = obj.Tb(:,1:7)-ones(nn,1)*obj.goal;
            tem2 = sqrt(sum(m.^2, 2));
            
            tem1=tem1+tem2;
            ellipseB = find(tem1<obj.cbest);
            
        end
        
        function Xnew = updateXsoln(obj)
            if norm(obj.Ta(1,1:7)-obj.start)<0.1
                X = obj.ellipseNodesA;
            else
                X= obj.ellipseNodesB;
            end
            Xnew=[];
            for i = 1:size(obj.Xsoln,1)
               if find(X==obj.Xsoln(i,1))
                   Xnew(end+1,:) = obj.Xsoln(i,:);
               end
            end
           
        end
        
     
        function soln = addNewSolution(obj,index1,index2)
            soln = obj.Xsoln;
            if norm(obj.Ta(1,1:7)-obj.start)<0.1
                soln(end+1,:) = [index1 index2];
            else
                soln(end+1,:) = [index2 index1];
            end
        end
        
        function [sign,newpoint,index,tree] = extend(obj,tree,ellipse,parentindex, point)
            if obj.cbest == Inf     
                if parentindex ==0             
                    [nearindex,len] = obj.nearest(tree,point);
                else
                    nearindex=parentindex;
                    len = norm(point-tree(nearindex,1:7));
                end
                flag1=0;
                if len>obj.MaxConnectionDistance
                    newpoint = tree(nearindex,1:7)+(point-tree(nearindex,1:7))*obj.MaxConnectionDistance/len;
                    len = obj.MaxConnectionDistance;
                else
                    newpoint=point;
                    flag1=1;
                end

                if ~ obj.is_collision(tree(nearindex,1:7), newpoint)
                    new_node = [newpoint nearindex len];
                    index = size(tree,1)+1;
                    tree(end+1,:) = new_node;
                    
                    if flag1 == 0
                        sign ="Advanced";
                    else
                        sign="Reached";
                    end
                else
                    sign="Trapped";
                    index=[];
                    newpoint=[];
              
                end
            else
                if parentindex == 0
                    %disp("nearindexellip");disp(ellipse');
                    %disp("nearindextree");disp(size(tree,1));
                    [nearindex,len] = obj.nearest(tree(ellipse,1:7),point);
                    %disp("parent=0 1");disp(nearindex);  
                    nearindex = ellipse(nearindex);
                    %disp("parent=0 2");disp(nearindex);
                else
                    nearindex = parentindex;
                    len = norm(tree(nearindex,1:7)-point);
                end
                
                flag1=0;
                if len>obj.MaxConnectionDistance
                    newpoint = tree(nearindex,1:7)+(point-tree(nearindex,1:7))*obj.MaxConnectionDistance/len;
                    len = obj.MaxConnectionDistance;
                else
                    newpoint=point;
                    flag1=1;
                end
                
                if ~ obj.is_collision(tree(nearindex,1:7), newpoint)
                    if flag1 == 0
                        sign ="Advanced";
                    else
                        sign="Reached";
                    end
                    index = size(tree,1)+1;
                    
                    %near
                    M = tree(ellipse,1:7);
                    nearlist = [];
                    %num = 1+size(M,1);
                    %r2=min(2000*log10(num)/num,obj.step.^2); 
                    r2 = 2*obj.MaxConnectionDistance^2;
                    M1 = M - ones(size(M,1),1)*newpoint;
                    M1 = sum(M1.^2,2);
                    list1 = find(M1<=r2);
                    disp(["list1num",size(list1,1)]);
                    for i = list1'
                        if ~obj.is_collision(M(i,1:7),newpoint)
                            nearlist(end+1,1)=i;
                        end
                    end
                    disp(["nearlistnum",size(nearlist,1)]);
                    
                    nearlist = ellipse(nearlist);
                    %choose best parent index
                    minlen=obj.cost(tree,nearindex)+len;
                    minindex = nearindex;
                    for i = nearlist'
                        templen = obj.cost(tree,i)+norm(tree(i,1:7)-newpoint);
                        if templen <minlen
                            minlen = templen;
                            minindex = i;
                        end
                    end 
                    %disp("best parent1");
                    %disp(minindex);
                    %disp("best parent2");disp(newpoint);
                    len = norm(tree(minindex,1:7)-newpoint);
                    %disp("best parent3");disp(len);
                    tree(end+1,:) = [newpoint minindex len];
                    
                    %rewire                  
                    for i = nearlist'
                        if obj.cost(tree, i) > norm(tree(i,1:7)-newpoint)+ minlen
                            tree(i,8) = index;
                            tree(i,9)= norm(tree(i,1:7)-newpoint);
                        end                  
                    end                  
                    
                else
                    sign="Trapped";
                    index=[];
                    newpoint=[];
                end                
            end      
        end
        
        function len = cost(~, tree, index)
           len = 0;
           while tree(index,8)~=0
               len = len+tree(index,9);
               index = tree(index,8);
           end
        end
        function [index,len] = nearest(~,tree, point)
            temp=tree(:,1:7)-ones(size(tree,1),1)*point;
            
            [~,index] = min(sum(temp.^2,2));
            %disp("nearest func");
            %disp(index);
            len = norm(point-tree(index,1:7));
        end
        
        function [cbest ,optimal_path] = caculateShortestPath(obj)
           
            cbest = obj.pre_cbest;
            optimal_path = obj.pre_path;
            num = size(obj.Xsoln,1);
            if num~= 0
                
                if norm(obj.Ta(1,1:7)-obj.start)<0.1
                    A=obj.Ta(:,1:9);
                    B=obj.Tb(:,1:9);
                else
                    A=obj.Tb(:,1:9);
                    B=obj.Ta(:,1:9);
                end
                [minlen, minpath] = obj.extract_path(A,B,obj.Xsoln(1,:));
                
                minindex = 1;
                for i=2:num
                    
                    [len, path] = obj.extract_path(A,B,obj.Xsoln(i,:));
                    if len < minlen
                        minlen = len;
                        minpath = path;
                        minindex=i;
                    end
                end
                [temlen, tempath] = obj.caculateOptimizationPath(minpath);
                if temlen < obj.pre_cbest
                    cbest = temlen;
                    optimal_path = tempath;
                end
                temsoln = [obj.Xsoln(1:minindex-1,:);obj.Xsoln(minindex+1:end,:)];
                s = size(temsoln,1);
                list = randperm(s,floor(s^(1/3)));
                
                for i=list
                    [~, path] = obj.extract_path(A,B,temsoln(i,:));
                    [temlen, tempath] = obj.caculateOptimizationPath(path);
                    if temlen < cbest
                        cbest = temlen;
                        optimal_path = tempath;
                    end
                end               
            end
            
        end
        
        
        function [len,path] = caculateOptimizationPath(obj,totalPath)
            
            mm=1;
            nn=3;
            path = totalPath(mm,:);
            while nn<=size(totalPath,1)
                if ~obj.is_collision(totalPath(mm,1:7),totalPath(nn,1:7))
                    nn=nn+1;
                else
                    path(end+1,:)  = totalPath(nn-1,:);
                    mm = nn-1;
                    nn = mm+2;
                end            
            end   
            
            path(end+1,:) = totalPath(end,:);
            len = 0;
            for i = 1:size(path,1)-1
                len = len+norm(path(i+1,1:7)-path(i,1:7));
            end
            
        end
        
        function flag = is_collision(obj,near_point, new_point)
           flag = obj.StateValidator.isMotionValid(near_point, new_point);
           flag  =~flag;
        end
           
%         function flag = in_obstacles(obj,point)
%             flag=0;
%             x=point(1,1);
%             y=point(1,2);
%             for i = 1:size(obj.obs,1)
%                 tem = obj.obs{i,1};
%                 if (tem(1,1)<x)&&(x<tem(2,1))&&(tem(1,2)<y)&&(y<tem(2,2))
%                     flag = 1;
%                     return;
%                 end
%             end
%         end

        function [len,final_path] = extract_path(~,treeA, treeB,soln)

            pathA = treeA(soln(1,1), 1:7);
            parent_index = treeA(soln(1,1),8);
            len = treeA(soln(1,1),9);
            %disp("Aparentindex00");disp(parent_index);
            while parent_index~= 0
                pathA(end+1,:) = treeA(parent_index,1:7);
                len = len+treeA(parent_index,9);
                parent_index = treeA(parent_index,8);
                %disp("Aparentindex11");disp(parent_index);
                
            end
            pathA=flip(pathA);

            pathB = treeB(soln(1,2), 1:7);
            parent_index = treeB(soln(1,2),8);
            len = len+treeB(soln(1,2),9);
            while parent_index~= 0
                pathB(end+1,:) = treeB(parent_index,1:7);
                len = len+treeB(parent_index,9);
                parent_index = treeB(parent_index,8);
            end
            final_path = [pathA;pathB(2:end,:)];
            
        end
        
        function point = mysample(obj, cbest)
            if obj.cbest ~= Inf
                tem = sqrt(cbest.^2-obj.cmin.^2)/2;
                r=[cbest/2, tem,tem,tem,tem,tem,tem];
                L=diag(r);
                point = obj.informedSample(L);
                while any(point<obj.jointBounds(:,1)) || any(point>obj.jointBounds(:,2))
                     point = obj.informedSample(L);
                end
                point = point';
            else
                point = obj.ss.sampleUniform();
            end
        end
        
        function randpoint = informedSample(obj,L)        
            
            x_ball = rand(7,1)*2 - 1;
            while norm(x_ball)>=1
                x_ball = rand(7,1)*2-1;
            end
            
            randpoint=obj.C*L*x_ball+obj.x_center;         
            
        end
    end
end