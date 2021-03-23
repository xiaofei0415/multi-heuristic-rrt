classdef tryrrt
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Xsoln=[];%index of Tree ,column vector
        cbest=Inf;
        %Tb=[];
        area=[0 0;100 100];
        %obs={[45 37;55 65];[45 15;55 34]};
        obs = {[25 25;32 60];[47 45;55 75];[65 25;75 55]};
        start=[10 50];
        goal=[90 50];
        Ta=[];
        Tb=[];
        step = 5;
        valid_len = 1;
        n=300;
        optimal_path=[];
        isInitialPathFound=0;
        ellipseNodesA = [];%index of Tree ,column vector
        ellipseNodesB = [];
        cmin=0;
        C=[];
        x_center=[];
        pre_cbest = Inf;
        pre_path=[];
    end
    
    methods
        function obj = tryrrt()
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj = plan(obj);
        end
        
        function obj =  plan(obj)
            obj.Ta=[obj.start 0 0];
            obj.Tb=[obj.goal 0 0];
            obj.cmin=norm(obj.goal-obj.start);
            obj.x_center=[(obj.start+obj.goal)/2,0]';
            a_l=[(obj.goal(1)-obj.start(1))/obj.cmin; (obj.goal(2)-obj.start(2))/obj.cmin; 0];
            id_t=[1,0,0];
            M=a_l*id_t;
            [U,~,Vh] = svd(M);
            obj.C=(U*diag([1,1,det(U)*det(Vh')]))*(Vh);
            
            for i=1:obj.n
                
                [obj.cbest,obj.optimal_path] = obj.caculateShortestPath();
                obj.pre_cbest = obj.cbest;
                obj.pre_path = obj.optimal_path;
                
                if obj.isInitialPathFound
                    [obj.ellipseNodesA, obj.ellipseNodesB] = obj.updateEllipseNodes();
                    obj.Xsoln= obj.updateXsoln();
                end
                randpoint = obj.informedSample(obj.cbest);
                while randpoint(1,1)<obj.area(1,1)|| randpoint(1,1)>obj.area(2,1)|| randpoint(1,2)<obj.area(1,2)||randpoint(1,2)>obj.area(2,2)
                    randpoint = obj.informedSample(obj.cbest);
                end
                [sign1, newpoint,index1, obj.Ta] = obj.extend(obj.Ta, obj.ellipseNodesA,0,randpoint);
                if sign1~="Trapped"
                    
                    [sign2, ~, index2,obj.Tb] = obj.extend(obj.Tb,obj.ellipseNodesB,0, newpoint);
                    %while sign2 == "Advanced"
                       % [sign2, ~, index2,obj.Tb] = obj.extend(obj.Tb,obj.ellipseNodesB,index2, newpoint);
                    %end
                    
                end
                
                if sign1 ~="Trapped" && sign2 == "Reached"
                    obj.Xsoln=obj.addNewSolution(index1, index2);
                    obj.isInitialPathFound = 1;
                end
                %swap
                temp=obj.Ta;
                obj.Ta = obj.Tb;
                obj.Tb= temp;
                
                if i == 100 || i == 200||i==300||i==500
                    plot(obj);
                end
                
            end
        end
        function plot(obj)
            figure;
            hold on;
            grid on;
            plot([0 0 100 100 0], [0 100 100 0 0],'c-','LineWidth', 1.5);%cyan
            %fill([35 25 25 75 75 65 65 35 35], [35 35 65 65 35 35 55 55 35],'c');
            for i = 1:size(obj.obs,1)
                tem=obj.obs{i,1};
                plot([tem(1,1),tem(1,1),tem(2,1),tem(2,1),tem(1,1)], [tem(1,2),tem(2,2),tem(2,2),tem(1,2),tem(1,2)],'c-','LineWidth', 1.5);%cyan
            end
            
            if abs(obj.Ta(1,1)-obj.start(1,1))<0.01 && abs(obj.Ta(1,2)-obj.start(1,2))<0.01
                Xa = obj.Ta(:,1:3);
                Xb= obj.Tb(:,1:3);
            else
                Xa= obj.Tb(:,1:3);
                Xb= obj.Ta(:,1:3);
            end
            
            for i=1:size(obj.Ta,1)
                if i==1
                    plot(Xa(1,1),Xa(1,2),'*r');
                else
                    index = Xa(i,3);
                    plot([Xa(i,1) Xa(index,1)],[Xa(i,2),Xa(index,2)], 'b-');
                    plot(Xa(i,1),Xa(i,2), 'k.', 'MarkerSize', 8);
                end
            end
            
            for i=1:size(Xb,1)
                if i==1
                    plot(Xb(1,1),Xb(1,2),'*b');
                else
                    index = Xb(i,3);
                    plot([Xb(i,1) Xb(index,1)],[Xb(i,2),Xb(index,2)], 'r-');
                    plot(Xb(i,1),Xb(i,2), 'k.', 'MarkerSize', 8);
                end
            end
            
            drawnow;
            
            if obj.isInitialPathFound           
                fprintf("obj.cbest:  ");disp(obj.cbest);        
                
                x1=obj.x_center(1);
                y1=obj.x_center(2);
                a1=obj.cbest/2;
                v=obj.goal-obj.start;
                b1=sqrt(obj.cbest.^2-obj.cmin.^2)/2;
                sita=0:pi/100:2*pi;
                fi1=atan2(v(2),v(1));
                %plot ellipse
                plot(x1+a1*cos(fi1)*cos(sita)-b1*sin(fi1)*sin(sita),y1+a1*sin(fi1)*cos(sita)+b1*cos(fi1)*sin(sita),'--g');
                %plot optimal path
                for i = 1:size(obj.optimal_path,1)-1
                    plot([obj.optimal_path(i,1),obj.optimal_path(i+1,1)],[obj.optimal_path(i,2),obj.optimal_path(i+1,2)],'y-','LineWidth', 1.5);
                end
                %plot all Xsoln point             
                for i = 1:size(obj.Xsoln,1)
                    in = obj.Xsoln(i,1);
                    plot(Xa(in,1),Xa(in,2),'og');
                end
                drawnow;
            else
                disp("no inital path yet");          
            end
            hold off;
        end
        
        function [ellipseA,ellipseB] = updateEllipseNodes(obj)
            %after first path found,update the ellipse nodes of tree a and
            %tree b ,where extend () works
            nn= size(obj.Ta,1);
            tem1=zeros(nn,1);
            m = obj.Ta(:,1:2)-ones(nn,1)*obj.start;
            for i = 1:nn
                tem1(i,1)=norm(m(i,:));
            end
            tem2=zeros(nn,1);
            m = obj.Ta(:,1:2)-ones(nn,1)*obj.goal;
            for i = 1:nn
                tem2(i,1)=norm(m(i,:));
            end
            tem1=tem1+tem2;
            ellipseA = find(tem1<obj.cbest);
            
            nn= size(obj.Tb,1);
            tem1=zeros(nn,1);
            m = obj.Tb(:,1:2)-ones(nn,1)*obj.start;
            for i = 1:nn
                tem1(i,1)=norm(m(i,:));
            end
            tem2=zeros(nn,1);
            m = obj.Tb(:,1:2)-ones(nn,1)*obj.goal;
            for i = 1:nn
                tem2(i,1)=norm(m(i,:));
            end
            tem1=tem1+tem2;
            ellipseB = find(tem1<obj.cbest);
            
        end
        
        function Xnew = updateXsoln(obj)
            if abs(obj.Ta(1,1)-obj.start(1,1))<0.01 && abs(obj.Ta(1,2)-obj.start(1,2))<0.01
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
            if abs(obj.Ta(1,1)-obj.start(1,1))<0.01 && abs(obj.Ta(1,2)-obj.start(1,2))<0.01
                soln(end+1,:) = [index1 index2];
            else
                soln(end+1,:) = [index2 index1];
            end
        end
        
        function [sign,newpoint,index,tree] = extend(obj,tree,ellipse,parentindex, point)
            if ~obj.isInitialPathFound       
                if parentindex ==0             
                    [nearindex,len] = obj.nearest(tree,point);
                else
                    nearindex=parentindex;
                    len = norm(point-tree(nearindex,1:2));
                end
                flag1=0;
                if len>obj.step
                    newpoint = tree(nearindex,1:2)+(point-tree(nearindex,1:2))*obj.step/len;
                    len = obj.step;
                else
                    newpoint=point;
                    flag1=1;
                end

                if ~ obj.is_collision(tree(nearindex,1:2), newpoint)
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
                    [nearindex,len] = obj.nearest(tree(ellipse,1:2),point);
                    %disp("parent=0 1");disp(nearindex);  
                    nearindex = ellipse(nearindex);
                    %disp("parent=0 2");disp(nearindex);
                else
                    nearindex = parentindex;
                    len = norm(tree(nearindex,1:2)-point);
                end
                
                flag1=0;
                if len>obj.step
                    newpoint = tree(nearindex,1:2)+(point-tree(nearindex,1:2))*obj.step/len;
                    len = obj.step;
                else
                    newpoint=point;
                    flag1=1;
                end
                
                if ~ obj.is_collision(tree(nearindex,1:2), newpoint)
                    if flag1 == 0
                        sign ="Advanced";
                    else
                        sign="Reached";
                    end
                    index = size(tree,1)+1;
                    
                    %near
                    M = tree(ellipse,1:2);
                    nearlist = [];
                    num = 1+size(M,1);
                    r2=min(2000*log10(num)/num,obj.step.^2);
                    for i = 1:size(M,1)
                        vector = M(i,1:2) - newpoint;
                        if vector(1,1)^2+vector(1,2)^2 <=r2 && ~obj.is_collision(M(i,1:2),newpoint)
                            nearlist(end+1,1)=i;
                        end
                    end
                    nearlist = ellipse(nearlist);
                    %choose best parent index
                    minlen=obj.cost(tree,nearindex)+len;
                    minindex = nearindex;
                    for i = nearlist'
                        templen = obj.cost(tree,i)+norm(tree(i,1:2)-newpoint);
                        if templen <minlen
                            minlen = templen;
                            minindex = i;
                        end
                    end 
                    %disp("best parent1");
                    %disp(minindex);
                    %disp("best parent2");disp(newpoint);
                    len = norm(tree(minindex,1:2)-newpoint);
                    %disp("best parent3");disp(len);
                    tree(end+1,:) = [newpoint minindex len];
                    
                    %rewire                  
                    for i = nearlist'
                        if obj.cost(tree, i) > norm(tree(i,1:2)-newpoint)+ minlen
                            tree(i,3) = index;
                            tree(i,4)= norm(tree(i,1:2)-newpoint);
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
           while tree(index,3)~=0
               len = len+tree(index,4);
               index = tree(index,3);
           end
        end
        function [index,len] = nearest(~,tree, point)
            temp=tree(:,1:2)-ones(size(tree,1),1)*point;
            temp2=temp.^2;
            [~,index] = min(temp2(:,1)+temp2(:,2));
            %disp("nearest func");
            %disp(index);
            len = norm(point-tree(index,1:2));
        end
        
        function [cbest ,optimal_path] = caculateShortestPath(obj)
           if ~ obj.isInitialPathFound         
               cbest = Inf;
               optimal_path=[];
           else
               cbest = obj.pre_cbest;
               optimal_path = obj.pre_path;
               if size(obj.Xsoln,1)~= 0
                  
                   if abs(obj.Ta(1,1)-obj.start(1,1))<0.01 && abs(obj.Ta(1,2)-obj.start(1,2))<0.01
                        A=obj.Ta(:,1:3);
                        B=obj.Tb(:,1:3);
                   else
                        A=obj.Tb(:,1:3);
                        B=obj.Ta(:,1:3);
                   end
                   for i=1:size(obj.Xsoln,1)
                       [len, path] = obj.caculateOptimizationPath(A,B,obj.Xsoln(i,:));
                       if len < cbest
                            cbest = len;
                            optimal_path = path;
                       end
                   end
               end
           end
        end
        
        
        function [len,path] = caculateOptimizationPath(obj,A,B,soln)
            totalPath = obj.extract_path(A, B,soln);
            mm=1;
            nn=3;
            path = totalPath(mm,:);
            while nn<=size(totalPath,1)
                if ~obj.is_collision(totalPath(mm,1:2),totalPath(nn,1:2))
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
                len = len+norm(path(i+1,1:2)-path(i,1:2));
            end
            
        end
        
        function flag = is_collision(obj,near_point, new_point)
            seg_len = obj.valid_len;
            len = norm(new_point - near_point);
            seg_num = floor(len/seg_len);
            remain = mod(len, seg_len);
            unit = (new_point-near_point)/len;
            for i = 1:seg_num
                temp_point = near_point + i*seg_len*unit;
                if obj.in_obstacles(temp_point)
                    flag = 1;
                    return;
                end

            end
            if remain ~= 0
                if obj.in_obstacles(new_point)
                    flag = 1;
                    return;
                end
            end

            flag = 0;%no collison
        end

        function flag = in_obstacles(obj,point)
            flag=0;
            x=point(1,1);
            y=point(1,2);
            for i = 1:size(obj.obs,1)
                tem = obj.obs{i,1};
                if (tem(1,1)<x)&&(x<tem(2,1))&&(tem(1,2)<y)&&(y<tem(2,2))
                    flag = 1;
                    return;
                end
            end
        end
        
        function final_path = extract_path(~,treeA, treeB,soln)

            pathA = treeA(soln(1,1), 1:2);
            parent_index = treeA(soln(1,1),3);
            %disp("Aparentindex00");disp(parent_index);
            while parent_index~= 0
                pathA(end+1,:) = treeA(parent_index,1:2);
                parent_index = treeA(parent_index,3);
                %disp("Aparentindex11");disp(parent_index);
            end
            pathA=flip(pathA);

            pathB = treeB(soln(1,2), 1:2);
            parent_index = treeB(soln(1,2),3);
            while parent_index~= 0
                pathB(end+1,:) = treeB(parent_index,1:2);
                parent_index = treeB(parent_index,3);
            end
            final_path = [pathA;pathB(2:end,:)];

        end
        
        
        function randpoint = informedSample(obj, cbest)        
            if obj.isInitialPathFound
                r=[cbest/2, sqrt(cbest.^2-obj.cmin.^2)/2, sqrt(cbest.^2-obj.cmin.^2)/2];
                L=diag(r);
                a=rand();
                b=rand();
                if b<a
                    tem=b;
                    b=a;
                    a=tem;
                end
                x_ball = [b*cos(2*pi*a/b);b*sin(2*pi*a/b);0];
                randpoint=obj.C*L*x_ball+obj.x_center; 
                randpoint = randpoint(1:2,1)';
            else
                randpoint(1,1) = rand*(obj.area(2,1)-obj.area(1,1))+obj.area(1,1);
                randpoint(1,2) = rand*(obj.area(2,2)-obj.area(1,2))+obj.area(1,2);
            end
        end
    end
end

