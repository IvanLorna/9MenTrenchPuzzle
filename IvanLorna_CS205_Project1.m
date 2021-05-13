fprintf("CS205 Project 1");

%note: this project is designed to be similar to the CS170 8 tile puzzle solver
%project, thus alot of the code here will look similar to my implementation
%of it

%define the map matrix that is the Trench
%it is defined as the win condition
%the 0 is the blank space

problem3 = [
    -1, 0, -1;
     0, 2,  1]

 solution3 = [
    -1, 0, -1;
     1, 2,  0]


problem5 = [
    -1,-1, 0, -1, -1;
     0, 2, 2,  1,  0]

solution5 = [
    -1,-1, 0, -1, -1;
     1, 2, 2,  0,  0]
 
solution5v2 = [
    -1,-1, 1, -1, -1;
     0, 2, 0,  2,  0]


problem7 = [
    -1,-1, 0,-1, 0, -1, -1;
     0, 2, 2, 2, 2,  1,  0]

solution7 = [
    -1,-1, 0,-1, 0, -1, -1;
     1, 2, 2, 2, 2,  0,  0]

solution7v2 = [
    -1,-1, 1,-1, 0, -1, -1;
     0, 0, 0, 2, 2,  2,  2]

solution7v3 = [
    -1,-1, 0,-1, 0, -1, -1;
     0, 2, 2, 1, 2,  2,  0]

problemO = [
    -1,-1, 0,-1, 0, -1, -1;
     0, 2, 3, 4, 5,  1,  0]

solutionO = [
    -1,-1, 0,-1, 0, -1, -1;
     1, 2, 3, 4, 5,  0,  0]


%concatenated to the right of each matrix is the additional information
% 0 indicates a space that is empty and can be moved into by any trench men
% 1 indicates the trench officer and must be moved to the leftmost tile
% 2 indicates the other non-officer trench men
%-1 indicates an obstacle. these tiles cannot be moved into or moved around
% top right '0' will indicate weight of the node

%i will not create the code that randomly generates a combination of the
%puzzle, because it may not be in the parity needed to reach the solution,
%starting states can be input manually.
%i make the assumption a solution exists for the input initial state

%if the depth of the problem is known, 
% we can use it for depth limitedearchs
%depth = 4

%this is the enumerator that will define the queueing function used by the
%algorithm
% 0-- Uniform Cost search
% 1-- A* with Misplaced Tile Heuristic
% 2-- A* with Manhattan Distance Hueristic

%note that Uniform Cost will be the default for input outside these values
%QUEUEING_FUNCTION = 1

%nodes is a 3D array representing the queue of matrix states 
%   nodes(:,:,1) = node with least weight / closest to solution
%when appending a new node:
%   nodes(:,:,size(nodes,3)+1) = solution
%to sort this queue by path weight:
%   [~,SortOrder] = sort(weights),3)
%   node(:,:,SortOrder)

%if you know how deep the answer is, make it a parameter as to avoid longer
%runtime than neccessary, useful for testing, enter 0 if unknown
max_depth = 0;

h1 = zeros([6,1]);
h2 = zeros([6,1]);
h3 = zeros([6,1]);


%%
itt = 1;

tic;
general_search(problem3, solution3, 0);
h1(itt) = toc;
itt = itt+1;

tic;
general_search(problem5, solution5, 0);
h1(itt) = toc;
itt = itt+1;

tic;
general_search(problem5, solution5v2, 0);
h1(itt) = toc;
itt = itt+1;

tic;
general_search(problem7, solution7, 0);
h1(itt) = toc;
itt = itt+1;

tic;
general_search(problem7, solution7v2, 0);
h1(itt) = toc;
itt = itt+1;

tic;
general_search(problem7, solution7v3, 0);
h1(itt) = toc;
itt = itt+1;
%%

%tic;
%general_search(problemO, solutionO, 0);
%h1(itt) = toc;

%%
itt = 1;

tic;
general_search(problem3, solution3, 1);
h2(itt) = toc;
itt = itt+1;

tic;
general_search(problem5, solution5, 1);
h2(itt) = toc;
itt = itt+1;

tic;
general_search(problem5, solution5v2, 1);
h2(itt) = toc;
itt = itt+1;

tic;
general_search(problem7, solution7, 1);
h2(itt) = toc;
itt = itt+1;

tic;
general_search(problem7, solution7v2, 1);
h2(itt) = toc;
itt = itt+1;

tic;
general_search(problem7, solution7v3, 1);
h2(itt) = toc;
itt = itt+1;
%%

%tic;
%general_search(problemO, solutionO, 1);
%h2(itt) = toc;
%%
itt = 1;

tic;
general_search(problem3, solution3, 2);
h3(itt) = toc;
itt = itt+1;

tic;
general_search(problem5, solution5, 2);
h3(itt) = toc;
itt = itt+1;

tic;
general_search(problem5, solution5v2, 2);
h3(itt) = toc;
itt = itt+1;

tic;
general_search(problem7, solution7, 2);
h3(itt) = toc;
itt = itt+1;

tic;
general_search(problem7, solution7v2, 2);
h3(itt) = toc;
itt = itt+1;
%%

%tic;
%general_search(problem7, solution7v3, 2);
%h3(itt) = toc;
%itt = itt+1;

%%
h1
h2
h3(6) = 43220
%%


%execute the general search function derived from given psuedocode
colors = ['r-','g-','b-'];

semilogy(h1,'r-','LineWidth', 1, 'MarkerSize', 2);
hold on;
semilogy(h2,colors(2),'LineWidth', 1, 'MarkerSize', 2);
semilogy(h3,colors(3),'LineWidth', 1, 'MarkerSize', 2);
legend('Uniform Cost Search', 'A* with Misplaced Tile', 'A* with Manhattan Distance' )
title("Figure 4: Line Plot of Runtime of Different Algorithms")
xlabel('Test #');
ylabel('Time Taken to Solve (sec)')
hold off;
%%
%Simple Problem
Simple_problem = [
    -1, 0, -1;
     0, 2,  1]

 Simple_solution = [
    -1, 0, -1;
     1, 2,  0]
 
 tic;
 general_search(Simple_problem,Simple_solution,1)
 
%%
 %Complex Problem
 Complex_problem = [
     -1, -1, 0, -1, 0, -1, -1;
      0,  2, 2,  2, 2,  1,  0]
Complex_solution = [
    -1, -1, 0, -1, 0, -1, -1;
     0,  1, 2,  2, 2,  2,  0]
 
general_search(Complex_problem,Complex_solution,1)
 
%%
qf = 1
plot(data(:,1,qf),data(:,2,qf), colors(qf), 'LineWidth', 1, 'MarkerSize', 2);
hold on;
qf = 2
plot(data(:,1,qf),data(:,2,qf), colors(qf), 'LineWidth', 1, 'MarkerSize', 2);
hold off;

plot(data(:,1,qf),data(:,2,qf), colors(qf), 'LineWidth', 1, 'MarkerSize', 2);
hold on;
%legend('A* with Manhattan Distance' )
title("Figure 5: Line Plot of Runtime of A* with Manhattan Distance");
xlabel('Complexity (in moves needed to solve)');
ylabel('Time Taken to Solve (sec)');
hold off;

%%
function general_search(problem,solution, QUEUEING_FUNCTION) %answer is the return value
   w = 0;    
    visited = problem;
    [nodes,w] = MAKE_QUEUE(problem,w,solution, QUEUEING_FUNCTION);%nodes is the queue of states that the algorithm will analyze
    %current_depth = 0;
    max_depth = 0;
    
    %current_q_sz = 1;
    max_q_sz = 1;
    
    while size(nodes,3) >= 1
        if nodes(:,:,1) == solution %solution check
            front_of_queue = nodes(:,:,1)
            fprintf("solution found\n");
            fprintf("puzzle shape: %dx%d\n",size(solution,1),size(solution,2))
            fprintf("queueing function: %d\n",QUEUEING_FUNCTION)
            fprintf( "largest weight: %d\n", max_depth);
            fprintf("nodes expanded: %d\n",size(visited,3));
            fprintf("max queue size: %d\n",max_q_sz);
            %answer = nodes(:,:,1);
            return;
        end
        
        if (size(visited,3) > 1) && (size(nodes,3) > 1) %list of previously visited nodes
            for i = 1:size(visited,3)
                if (nodes(:,:,1) == visited(:,:,i))
                    nodes = nodes(:,:,2:size(nodes,3));
                end
            end
        end
        
        if max_depth < w(1) %check for max depth
            max_depth = w(1);
        end
        
        if max_q_sz < size(nodes,3) %check for largest queue size
            max_q_sz = size(nodes,3);
        end
        
        [nodes, w,visited] = UPDATE_QUEUE(nodes,w,solution,QUEUEING_FUNCTION,visited); %update queue
    end
    fprintf("\nfailed to find solution\n")
end

function [q,w] = CHECK_VISITED(q,w,v)
    k = size(q,3);
    l = size(v,3);
    index = [];
    for i =1:k
        for y = 1:l
            if q(:,:,i) == v(:,:,y)
                %q(:,:,i) = [];
                %w(i) = [];
                index = cat(1,index,i);
            end
            
        end
    end
    
    q = q(:,:,setdiff(1:k,index));
    w = w(setdiff(1:k,index));
end

%queue updating helper function
%makes all posible "next moves" given a state, doesnt update the weight
function  [q,w] = MAKE_QUEUE(problem,w,solution,QUEUEING_FUNCTION)
    q = problem;
    for i = 1:size(problem,1)
        for y = 1:size(problem,2)
            if problem(i,y) == 0
                newnodes = MAKE_LEGAL_MOVES(problem,i,y);
                q = cat(3,q,newnodes);
            end
        end
    end
    
    %update weights of new nodes
    q = q(:,:,2:size(q,3));
    w = CALC_WEIGHTS(q,ones([size(q,3),1])*w,solution,QUEUEING_FUNCTION);
    return;
end

function q = MAKE_LEGAL_MOVES(problem,i,y)
    moves_made = 0;
    if i == 1 %if on the top row, the only legal move is swapping with the tile below
        moves_made = moves_made +1;
        q(:,:,moves_made) = problem;
        q(i,y,moves_made) = q(2,y,moves_made);
        q(2,y,moves_made) = 0;
    else     %else, move left right up if possible
        if y > 1
            moves_made = moves_made +1;
            q(:,:,moves_made) = problem;
            q(i,y,moves_made) = q(i,y-1,moves_made);
            q(i,y-1,moves_made) = 0;
        end
        if y < size(problem,2)
            moves_made = moves_made +1;
            q(:,:,moves_made) = problem;
            q(i,y,moves_made) = q(i,y+1,moves_made);
            q(i,y+1,moves_made) = 0;
        end
        if q(i-1,y) ~= -1
            moves_made = moves_made +1;
            q(:,:,moves_made) = problem;
            q(i,y,moves_made) = q(i-1,y,moves_made);
            q(i-1,y,moves_made) = 0;
        end
    end
    return;
end

%weight recalculating helper function
%this is where queueing function comes into play
%calculates weight differently depending on input queueing function
function w = CALC_WEIGHTS(q,w,solution,QUEUEING_FUNCTION)
    switch QUEUEING_FUNCTION
        case 1
            w = MISPLACED_TILE(q,solution);
            
        case 2
            w = MANHATTAN_DISTANCE(q,solution);
        
        otherwise %Uniform Cost Search is used if 0 or an undefined value
            w = w + 1;
    end
    return;
end

function w = MANHATTAN_DISTANCE(q,solution)
    w = zeros([size(q,3),1]);
    for i = 1:size(q,3)
        for ys = 1:size(solution,1)
            for xs = 1:size(solution,2)
                for yp = 1:size(q,1)
                    for xp = 1:size(q,2)
                        if solution(ys,xs) == q(yp,xp,i)
                            w(i) = w(i) + abs(ys-yp + xs-xp);
                        end
                    end
                end
            end
        end
    end
end 

function w = MISPLACED_TILE(q,solution)
    w = zeros([size(q,3),1]);
    for i = 1:size(w,1)
        sum = 0;
        for y = 1:size(solution,1)
            for x = 1:size(solution,2)
                if q(y,x,i) ~= solution(y,x)
                    sum = sum + 1;
                end
            end
        end
        w(i) = sum+1;
        
    end

end

function [q,w,v] = UPDATE_QUEUE(nodes,w, solution, QUEUEING_FUNCTION,v)
    %expand on top node
    [nq,nw] = MAKE_QUEUE(nodes(:,:,1),w(1),solution, QUEUEING_FUNCTION);
    %concatenate new nodes to queue
    q = cat(3,nodes,nq);
    w =[w;nw];
    %remove top node

    front_of_queue = nodes(:,:,1) %uncomment to have popped nodes printed for output
    v= cat(3,v,q(:,:,1));
    
    [q,w] = CHECK_VISITED(q,w,v);
    
    %sort nodes by least weight to greatest
    [~,SortOrder] = sort(w,1);
    q = q(:,:,SortOrder);
    w = w(SortOrder);

    %q,w is returned with new sorted nodes 
    return;
end