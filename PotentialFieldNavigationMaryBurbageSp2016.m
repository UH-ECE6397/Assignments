function PotentialFieldNavigationMaryBurbageSp2016
% TODO: draw line between repulsion point
% TODO: animate the random walk
% A path planner for an n-link planar robot arm moving amoung polygonal obstacles.
%
% Based off chapter 5.2 in "Robot Modeling and Control" by Spong, Hutchinson,
% and Vidyasagar
%
% Aaron T Becker, 04-13-2016, atbecker@uh.edu
%
%This solution uses code by Mary Burbage
%
%  Items to complete are marked with "TODO:"
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% A FUN 4-link example
linkLen = [1;1;1;1];  %lengths of each link
q = [pi/2;0;pi/4;0]; %robot configuration (set)
qGoal = [-pi/2;0;-pi/2;0];
rhoNot = 1.5; %: defines the distance of influence of the obstacle

% %% ACTUAL ASSIGNMENT
% linkLen = [2;2];  %lengths of each link
% q = [pi/2;pi/4]; %robot configuration (set)
% qGoal = [-pi/2;-pi/2];
% rhoNot = 1/2; %: defines the distance of influence of the obstacle

%%% this makes the random generator the same each run.  Useful for
%%% debugging
s = RandStream('mt19937ar','Seed',1);
RandStream.setGlobalStream(s);


moveWeight = flipud(cumsum(flipud(linkLen))); %moving a base link moves all futher links, so moves of base links are penalized more than distal links
%q = rand(numel(linkLen),1)*2*pi;  %robot configuration (random)

%qGoal = rand(numel(linkLen),1)*2*pi; %TODO: some sort of check to make sure this isn't intersecting an obstacle

oGoal = computeOrigins(qGoal); %robot goal DH origins
ptObstacles = [1.5,2;  -0.5,0.5]; %locations of point obstacles
q_recent = zeros(numel(q),4); %recent positions of robot arm

% Parameters  (students should change these)
zeta = flipud(.1*cumsum(ones(numel(q),1)));%zeta: vector parameter that scales the forces for each degree-of-freedom
alpha = 0.02; %step size for each iteration.  In motion planning problems, the choice for alpha is often made on an ad hoc or empirical basis, such as the distance to the nearest obstacle or goal
d = 0.5; %d: the distance that defines the transition from conic to parabolic
eta = ones(numel(q),1); %eta: vector parameter that scales the repulsive forces for each degree-of-freedom

inLocalMinimum = false;
t = 100;  %how many random steps to take?
v = pi/8; % maximum random value at each step ( link length is max step size, 2*v)
eps_m = 0.02; %epsilon_m:  limit for step sizes for determining local minima
isPolygonObs = false; % if true, uses polygonal obstacles


%setup figure
figure(1);clf;
r = sum(linkLen);
rectangle('Position',[-r,-r,2*r,2*r],'Curvature',1,'FaceColor',[.9 .9 1]) %robot workspace
axis equal
hold on
% draw obstacles
if isPolygonObs
    ptObstacles = [1,2; -3,-1]; %#ok<UNRCH>
    polyObs1 = repmat(ptObstacles(1,:),3,1)+[.5000    0.8660
        -1.0000    0.0000
        0.5000   -0.8660];
    
    polyObs2 = repmat(ptObstacles(2,:),4,1)+[-1/2    -1
        -1/2    1
        1/2   1
        1/2 -1];
    
    p1 = fill(polyObs1(:,1),polyObs1(:,2),'r');
    set(p1,'linewidth',40,'EdgeColor',[1 .8 .8] );
    fill(polyObs1(:,1),polyObs1(:,2),'r');
    p2 = fill(polyObs2(:,1),polyObs2(:,2),'r');
    set(p2,'linewidth',40,'EdgeColor',[1 .8 .8] );
    fill(polyObs2(:,1),polyObs2(:,2),'r');
    
else
    for j = 1:numel(ptObstacles(:,2))
       rectangle('Position',[ptObstacles(j,1)-rhoNot,ptObstacles(j,2)-rhoNot,2*rhoNot,2*rhoNot],'Curvature',1,'FaceColor',[1 .8 .8],'LineStyle','none')
    end
    plot(ptObstacles(:,1),ptObstacles(:,2),'r*')
end
%draw robot
hGline = line([0,0],[0,1],'color','g','linewidth',4);
hGpts = plot([0,0],[0,1],'og','markersize',12);
hRline = line([0,0],[0,1],'color','b','linewidth',4);
hRpts = plot([0,0],[0,1],'ob','markersize',12);
harr= quiver(0,0,1,2,'color',[0,0.8,0],'linewidth',2);
hrep= quiver(0,0,1,2,'color',[0,0,1],'linewidth',2);
hold off
hTitle = title(num2str(0));
set(hTitle,'fontsize',24);
set(gca,'fontsize',20)
maxIters = 2000;
hClosestLine = zeros(numel(ptObstacles(:,2))*numel(linkLen),1);
for j = 1:numel(ptObstacles(:,2))*numel(linkLen);
    hClosestLine(j) = line([0,0],[0,1],'color','m');
    set(hClosestLine(j),'marker','.');
end
Freps = zeros(numel(ptObstacles(:,2))*numel(linkLen),2);
RepsPts = zeros(numel(ptObstacles(:,2))*numel(linkLen),2);

updateArm(oGoal,hGline,hGpts);
for iteration = 1:maxIters  %each iteration performs gradient descent one time
    %calulate error
    qErr = sum(atan2(sin(q-qGoal),cos(q-qGoal)).^2);
    oR = computeOrigins(q);
    for j = 1:numel(ptObstacles(:,1))
        [Frep,clpts] = frepPtFloatingPoint(q, ptObstacles(j,:), eta, rhoNot);
        for n = 1:numel(q)
            idx = (j-1)*numel(q)+n;
            set(hClosestLine(idx), 'Xdata',[clpts(n,1),ptObstacles(j,1)],'Ydata',[clpts(n,2),ptObstacles(j,2)]);
            Freps(idx,:) = Frep(n,:);
            RepsPts(idx,:) =clpts(n,:);
        end
    end
    Fatt = fatt(q, oGoal,zeta,d);
 
    
    %update drawing
    
    updateArm(oR,hRline,hRpts);
    set(harr, 'Xdata',oR(:,1),'Ydata',oR(:,2),'Udata',Fatt(:,1),'Vdata',Fatt(:,2));
    set(hrep, 'Xdata',RepsPts(:,1),'Ydata',RepsPts(:,2),'Udata',Freps(:,1),'Vdata',Freps(:,2));
    set(hTitle,'String',[num2str(iteration),' of ', num2str(maxIters), ' error=',num2str(qErr)])
    drawnow
    if qErr < 0.001
        break
    end
    
    %map the workspace forces to joint torques (5.2.3)
    tau = zeros(numel(q),1);
    for ic = 1:numel(q)
        tau = tau +  Jv(q,ic)'*Fatt(ic,:)';
    end
    % we need a new Jv for floating repulsion points
    for jc = 1:numel(ptObstacles(:,2))
        for ic = 1:numel(q)
            idx = (jc-1)*numel(q)+ic;
            tau = tau +  JvArbPt(q,ic,RepsPts(idx,:))'*Freps(idx,:)';
        end
    end
    
    %Gradient descent algorithm, page 179
    amin = min(alpha, 1/2*max(abs(q-qGoal)));
    q = q+amin*tau/norm(tau);
    
    %TODO Task 3  (5pts) detect a local minimum
    q_recent = [q_recent(:,2:end) q];
    if iteration > 3
        inLocalMinimum = (dist(q_recent(:,4),qGoal) > eps_m && ...
            dist(q_recent(:,1),q_recent(:,2)) < eps_m && ...
            dist(q_recent(:,1),q_recent(:,3)) < eps_m && ...
            dist(q_recent(:,1),q_recent(:,4)) < eps_m);
    end
    
    if inLocalMinimum
        %TODO: Task 4  (5pts) random walk:
        %execute a random walk.  If it results in collision, do not apply
        %it.
        qprime = q;
        for j =1:t
            %generate random +/-1 for each link
            vsign = -1+2*rand([numel(q) 1]);
            %vsign(vsign<0.5)=-1;
            %vsign(vsign>=0.5)=1;
            vsign = vsign./moveWeight;
            %only assign random walk steps to joints that are further than
            %eps_m from goal
            vsign((q_recent(:,1)-qGoal(:,1))<eps_m) = 0;
            %generate a step
            qstep = qprime+vsign*v;
            %check for a collision for each obstacle
            inCollision = false; % if any joint is in collision (defined as being too close to an obstacle) the move is rejected.
            for k = 1:numel(ptObstacles(:,1))
                if checkCollision(qstep,ptObstacles(k,:))
                    inCollision=true;
                end
            end
            if inCollision == false
                qprime = qstep;
                oR = computeOrigins(qprime);
                updateArm(oR,hRline,hRpts);
                drawnow
            end
        end
        q = qprime;
    end
    
    %pause(0.5)
end

    function J = Jv(q,ic) %page 177
        o = computeOrigins(q);
        Augo = [0,0;o];  %add frame 0  (augmented origin)
        % J = [z0 x (o_c - o_0),  z1 x (o_c - o_2), ...., z_(n-1) x (o_c - o_(n-1))
        J = zeros(2,numel(q));
        for c = 1:ic
            oDiff=  Augo(ic+1,:)-Augo(c,:);
            J(1,c) = -oDiff(2);
            J(2,c) = oDiff(1);
        end
    end

    function J = JvArbPt(q,ic,RepsPt) %page 177
        o = computeOrigins(q);
        o(ic,:) = RepsPt;
        Augo = [0,0;o];  %add frame 0  (augmented origin)
        % J = [z0 x (o_c - o_0),  z1 x (o_c - o_2), ...., z_(n-1) x (o_c - o_(n-1))
        J = zeros(2,numel(q));
        for c = 1:ic
            oDiff=  Augo(ic+1,:)-Augo(c,:);
            J(1,c) = -oDiff(2);
            J(2,c) = oDiff(1);
        end
    end
    

    function Fvec =  fatt(q, oGoal,zeta,d)
        %fatt computes the forces that attract each DH frame origin to their goal
        %configurations, given by equation 5.4 in RD&C
        %q: configuration of the arm
        %oGoal: goal position of each DH frame origin
        %zeta: vector parameter that scales the forces for each degree-of-freedom
        %d: the distance that defines the transition from conic toparabolic
        Fvec = zeros(numel(q),2); %Force vector to attract each origin to the goal
        o = computeOrigins(q); % o is a vector of the origins for DH frames of a planar robot arm.
        
        for i = 1:numel(q) % compute attractive force for each origin
            err = dist(o(i,:),oGoal(i,:));
            if err < d
                Fvec(i,:)= -zeta(i)*( o(i,:)-oGoal(i,:)  );
            else
                Fvec(i,:)= -d*zeta(i)*( o(i,:)-oGoal(i,:)  )/err;
            end
        end
    end

    function o = computeOrigins(q)
        %Computes o, the (x,y) coordinate of the DH frame for each link in q
        qSum = cumsum(q);
        oDelta = [linkLen,linkLen].*[cos(qSum),sin(qSum)];
        o = cumsum(oDelta);
    end

    function Fvec = frepPt(q, pObstacle, eta, rhoNot) %#ok<DEFNU>
        % TODO: Task 1  (5pts) repulsion from point obstacle
        %frepPt computes the forces that repel each DH frame origin from a point
        %at positon pObstacle, given by equation 5.6 & 5.7 in RD&C
        %q: configuration of the arm
        %pObstacle: xy position of the point obstacle
        %eta: vector parameter that scales the forces for each degree-of-freedom
        %rhoNot: defines the distance of influence of the obstacle
        Fvec = zeros(numel(q),2); %Force vector to repulse each origin from the obstacle
        o = computeOrigins(q); % o is a vector of the origins for DH frames of a planar robot arm.
        
        for i = 1:numel(q) % compute repulsive force for each origin
            rho = dist(o(i,:),pObstacle);
            if rho < rhoNot
                delRho = (o(i,:)-pObstacle)/rho;
                Fvec(i,:)= eta(i)*(1/rho - 1/rhoNot)*(1/rho^2)*delRho;
            end
        end
    end

    function p = findClosestPointOnLine(lineStart, lineEnd, pt)
        %calculate the point on a line that is closest to another point
        
        %calculate the line vector
        lineSeg = lineStart-lineEnd;
        %calculate the vector from the point to the end of the line
        ptToLine = pt-lineEnd;
        
        %calculate the dot products of the line to the point vector
        %and the line to itself
        c1 = dot(ptToLine,lineSeg,2);
        c2 = dot(lineSeg,lineSeg,2);
        %if c1 <= 0, the point is closer to the end of the line segment
        if c1 <= 0
            p = lineEnd;
            return;
            %if c2 <= c1, the point is closer to the start of the line segment
        elseif c2 <= c1
            p = lineStart;
            return;
        end
        %otherwise, calculate the point along the line that is closest
        %to the point
        lambda = c1/c2;
        p = lineEnd + lambda*lineSeg;
    end

    function [Fvec, p,rho] =  frepPtFloatingPoint(q, pObstacle, eta, rhoNot)
        % Task 2  (Graduate students 5pts, Undergrads, 5pts E.C.):
        %computes the forces that repel a point on the link that is closest to any workspace obstacle
        %at positon pObstacle, given by equation 5.6 & 5.7 in RD&C
        %q: configuration of the arm
        %pObstacle: xy position of the point obstacle
        %eta: vector parameter that scales the forces for each degree-of-freedom
        %rhoNot: defines the distance of influence of the obstacle
        Fvec = zeros(numel(q),2); %Force vector to repulse each link from the obstacle
        o = computeOrigins(q); % o is a vector of the origins for DH frames of a planar robot arm.
        p = zeros(numel(q),2);
        rho = zeros(numel(q),2);
        
        for i = 1:numel(q) % compute repulsive force for each link
            %find the nearest point on the link to the obstacle
            if i == 1
                p(i,:) = findClosestPointOnLine([0 0],o(i,:),pObstacle);
            else
                p(i,:) = findClosestPointOnLine(o(i-1,:),o(i,:),pObstacle);
            end
            %calculate the distance from the floating point to the obstacle
            rho(i) = dist(p(i,:),pObstacle);
            %if it is inside rho0, calculate the force
            if rho(i) < rhoNot
                delRho = (p(i,:) -pObstacle)/rho(i);
                Fvec(i,:) = eta(i)*(1/rho(i) - 1/rhoNot)*(1/rho(i)^2)*delRho;
            end
        end
    end

    function collision =  checkCollision(q, pObstacle)
        %TODO: this should compare the swept out region of the robot and
        %ensure that region does not include the obstacle
        %computes the point on the link that is closest to any workspace obstacle
        %at position pObstacle, if it is at the same location as the
        %obstacle, the two are in collision
        %q: configuration of the arm
        %pObstacle: xy position of the point obstacle
        %eps_min: distance from a point obstacle that the robot must be to
        %keep out of collision
        eps_min = 0.5;  %TODO:  this could be v*LinkLength
        o = computeOrigins(q); % o is a vector of the origins for DH frames of a planar robot arm.
        collision = false;
        
        for i = 1:numel(q) % check whether the floating points are on the obstacles
            %find the nearest point on the link to the obstacle
            if i == 1
                p = findClosestPointOnLine([0 0],o(i,:),pObstacle);
            else
                p = findClosestPointOnLine(o(i-1,:),o(i,:),pObstacle);
            end
            rho = dist(p,pObstacle);
            if rho <= eps_min
                % if the point is within eps_min of the obstacle, it is
                % in collision
                collision = true;
                return;
            end
        end
    end

    function updateArm(o,hline,hpts) %redraws arm
        set(hline, 'xdata',[0;o(:,1)], 'ydata',[0;o(:,2)]);
        set(hpts, 'xdata',[0;o(:,1)], 'ydata',[0;o(:,2)]);
    end

    function d = dist(a,b)% norm 2 distance between two vectors
        d=sum((a-b).^2).^.5;
    end
end