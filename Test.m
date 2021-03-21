function  Test
myrrt();
end

function  myrrt
start_point = [50 20];
goal_point = [50 80];
seg_length = 5;
% [x_position y_position parent_node_row_num from_start_to_this_total_path_length]
treeA = [start_point 0 0];
treeB=[goal_point 0 0];
for i=1:4000
   rand_point=rand(1,2)*100;
   [index, len]= nearest(treeA, rand_point);
   
   if len>seg_length
        new_point = treeA(index,1:2)+(rand_point-treeA(index,1:2))*seg_length/len;
        len = seg_length;
   else
       new_point=rand_point;
   end
   
   if ~ is_collision(treeA(index,1:2), new_point)
       new_node = [new_point index len+treeA(index,4)];
       %tree=[tree; new_node];
       treeA(end+1,:) = new_node;
   else
       [treeA,treeB]=swap(treeA,treeB);
       continue;
   end
   
   [index, len]= nearest(treeB, new_point);
   if len>seg_length
        new_point2 = treeB(index,1:2)+(new_point-treeB(index,1:2))*seg_length/len;
        len = seg_length;
   elseif len == 0%attension !maybe happen
       final_path = extract_path(treeA, treeB(1:index,:),2);
       break;
   else
       new_point2=new_point;     
   end
   
   if ~ is_collision(treeB(index,1:2), new_point2)
       new_node = [new_point2 index len+treeB(index,4)];
       treeB(end+1,:) = new_node;
       connect_flag = is_connected(new_point,new_point2);
       
       if connect_flag
           final_path = extract_path(treeA, treeB,connect_flag);
           break;
       end     
       
   end
   
    [treeA,treeB]=swap(treeA,treeB);
    
end

figure;
hold on;
plot_world();
plot_trees(treeA,treeB);
if connect_flag
   fprintf("path found %d inters %d path_segs\n",i, size(final_path,1));
   plot_final_path(final_path);
   
else
    fprintf("path not found after about %d inters\n",i);
end
hold off;
end

function  plot_world()
plot([0 0 100 100], [0 100 100 0],'c-');%cyan
fill([35 25 25 75 75 65 65 35 35], [35 35 65 65 35 35 55 55 35],'c');
end

function plot_trees(treeA,treeB)
%tree hardly has only one elements except that two root very close and fist sample the goal

for i=1:size(treeA,1)
    if i==1
        plot(treeA(1,1),treeA(1,2),'or');
    else
        index = treeA(i,3);
        plot([treeA(i,1) treeA(index,1)],[treeA(i,2),treeA(index,2)], 'r-');
        plot(treeA(i,1),treeA(i,2), 'k.', 'MarkerSize', 8);
    end
end

for i=1:size(treeB,1)
    if i==1
        plot(treeB(1,1),treeB(1,2),'ob');
    else
        index = treeB(i,3);
        plot([treeB(i,1) treeB(index,1)],[treeB(i,2),treeB(index,2)], 'b-');
        plot(treeB(i,1),treeB(i,2), 'k.', 'MarkerSize', 8);
    end
end

end

function plot_final_path(final_path)
plot(final_path(:,1), final_path(:,2), 'y-', 'LineWidth', 1.5);    
end


function final_path = extract_path(treeA, treeB,connect_flag)
goal_point = [50,80];
if treeA(1,1) == goal_point(1) && treeA(1,2) == goal_point(2)
    %not ok to write like "tree(1,1:2)==goal_point"
    [treeA,treeB]=swap(treeA, treeB);
end

pathA = treeA(end, 1:2);
parent_index = treeA(end,3);
while parent_index~= 0
    pathA(end+1,:) = treeA(parent_index,1:2);
    parent_index = treeA(parent_index,3);
end
[row,col] = size(pathA);
tempA = zeros(row,col);
for i=1:row
tempA(i,:) = pathA(row-i+1,:);
end

pathB = treeB(end, 1:2);
parent_index = treeB(end,3);
while parent_index~= 0
    pathB(end+1,:) = treeB(parent_index,1:2);
    parent_index = treeB(parent_index,3);
end

if connect_flag ==1
    final_path = [tempA; pathB];
elseif connect_flag == 2
    final_path = [tempA;pathB(2:end,:)];
end

end

function [outA, outB] = swap(inA, inB)
outA=inB;
outB=inA;
end

function [index,len] = nearest(tree, point)
temp=tree(:,1:2)-ones(size(tree,1),1)*point;
temp2=temp.^2;
[~,index] = min(temp2(:,1)+temp2(:,2));
len = norm(point-tree(index,1:2));
end

function flag = is_collision(near_point, new_point)
seg_len = 1;
len = norm(new_point - near_point);
seg_num = floor(len/seg_len);
remain = mod(len, seg_len);
unit = (new_point-near_point)/len;
for i = 1:seg_num
    temp_point = near_point + i*seg_len*unit;
    if in_obstacles(temp_point)
        flag = 1;
        return;
    end
    
end
if remain ~= 0
    if in_obstacles(new_point)
        flag = 1;
        return;
    end
end

flag = 0;%no collison
end

function flag = in_obstacles(point)
flag=0;
x=point(1,1);
y=point(1,2);
if (25<x)&&(x<75)&&(35<y)&&(y<65)&&~((35<x)&&(x<65)&&(35<y)&&(y<55))
    flag = 1;
end
end

function flag = is_connected(point1,point2)
seg_len = 5;
len = norm(point1-point2);
if len>seg_len
  flag  = 0;
  return;
elseif len == 0
    flag = 2;
    return;
else
    if is_collision(point1, point2)
        flag = 0;
    else
        flag = 1;
    end
end

end

