function PotentialFieldNavigation
% A path planner for an n-link planar robot arm moving amoung polygonal obstacles.
%
% Based off chapter 5.2 in "Robot Modeling and Control" by Spong, Hutchinson,
% and Vidyasagar
%
% Aaron T Becker, 04-13-2016, atbecker@uh.edu
%
%  Items to complete are marked with "TODO:"
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
linkLen = [2;2;];  %lengths of each link
%q = rand(numel(linkLen),1)*2*pi;  %robot configuration (random)
q = [pi/2;pi/4]; %robot configuration (set)
%qGoal = rand(numel(linkLen),1)*2*pi; %TODO: some sort of check to make sure this isn't intersecting an obstacle
qGoal = [-pi/2;-pi/2];
oGoal = computeOrigins(qGoal); %robot goal DH origins
ptObstacles = [1.5,2;  -0.5,0.5]; %locations of point obstacles


% Parameters  (students should change these)
zeta =   flipud(.1*cumsum(ones(numel(q),1)));%zeta: vector parameter that scales the forces for each degree-of-freedom
alpha = 0.02; %step size for each iteration.  In motion planning problems, the choice for alpha is often made on and adhoc or empirical basis, such as the distance to the nearest obstacle or goal
d = 0.5; %d: the distance that defines the transition from conic toparabolic
eta = ones(numel(q),1); %eta: vector parameter that scales the repulsive forces for each degree-of-freedom
rhoNot = 1/2; %: defines the distance of influence of the obstacle
inLocalMinimum = false;
t = 5;  %how many random steps to take?
v = pi/10; % maximum random value at each step

%setup figure
figure(1);clf;
r = sum(linkLen);
rectangle('Position',[-r,-r,2*r,2*r],'Curvature',1,'FaceColor',[.9 .9 1]) %robot workspace
axis equal
hold on
% draw obstacles
for j = 1:numel(ptObstacles(:,2))
    rectangle('Position',[ptObstacles(j,1)-rhoNot,ptObstacles(j,2)-rhoNot,2*rhoNot,2*rhoNot],'Curvature',1,'FaceColor',[1 .8 .8],'LineStyle','none')
end
plot(ptObstacles(:,1),ptObstacles(:,2),'r*')
%draw robot
hGline = line([0,0],[0,1],'color','g');
hGpts = plot([0,0],[0,1],'og');
hRline = line([0,0],[0,1],'color','b');
hRpts = plot([0,0],[0,1],'ob');
harr= quiver(0,0,1,2,'color','r');
hold off
hTitle = title(num2str(0));
maxIters = 2000;

for iteration = 1:maxIters
    %calulate error
    qErr = sum(atan2(sin(q-qGoal),cos(q-qGoal)).^2);
    oR = computeOrigins(q);
    frep = zeros([numel(q),2]);
    for j = 1:numel(ptObstacles(:,1))
        frep = frep + frepPt(q, ptObstacles(j,:), eta, rhoNot);
    end
    Fvec =  fatt(q, oGoal,zeta,d) + frep;
    
    %update drawing
    updateArm(oGoal,hGline,hGpts);
    updateArm(oR,hRline,hRpts);
    set(harr, 'Xdata',oR(:,1),'Ydata',oR(:,2),'Udata',Fvec(:,1),'Vdata',Fvec(:,2));
    set(hTitle,'String',[num2str(iteration),' of ', num2str(maxIters), ' error=',num2str(qErr)])
    drawnow
    if qErr < 0.001
        break
    end
    
    %map the workspace forces to joint torques (5.2.3)
    tau = zeros(numel(q),1);
    for ic = 1:numel(q)
        tau = tau+Jv(q,ic)'* Fvec(ic,:)';
    end
    
    %Gradient descent algorithm, page 179
    q = q+alpha*tau/norm(tau);
    
    %TODO Task 3  (5pts) detect a local minimum
    
    
    if inLocalMinimum
        %TODO: Task 4  (5pts) random walk:  
        %execute a random walk.  If it results in collision, do not apply
        %it.
        qprime = q;
        for j =1:t
            qprime = qprime+0;
            %check for collision
        end
        q = qprime;
    end
    
    %pause(0.5)
end

    function J = Jv(q,ic) %page 177
        o = computeOrigins(q);
        Augo = [0,0;o];  %add frame 0
        % J = [z0 x (o_c - o_0),  z1 x (o_c - o_2), ...., z_(n-1) x (o_c - o_(n-1))
        J = zeros(2,numel(q));
        for c = 1:ic
            oDiff=  Augo(ic+1,:)-Augo(c,:);
            J(1,c) = -oDiff(2);
            J(2,c) = oDiff(1);
        end
        
        %topRowManipJacobian =  fliplr([-o(:,2)';o(:,1)']);
    end

    function Fvec =  fatt(q, oGoal,zeta,d)
        %computes the forces that attract each DH frame origin to their goal
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
        %o = zeros(numel(q),2); %(x,y) coordinate of every origin frame
        qSum = cumsum(q);
        oDelta = [linkLen,linkLen].*[cos(qSum),sin(qSum)];
        o = cumsum(oDelta);
    end

    function Fvec =  frepPt(q, pObstacle, eta, rhoNot)
        % TODO: Task 1  (5pts) repulsion from point obstacle
        %computes the forces that repel each DH frame origin from a point
        %at positon pObstacle, given by equation 5.6 & 5.7 in RD&C
        %q: configuration of the arm
        %pObstacle: xy position of the point obstacle
        %eta: vector parameter that scales the forces for each degree-of-freedom
        %rhoNot: defines the distance of influence of the obstacle
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

function Fvec =  frepPtFloatingPoint(q, pObstacle, eta, rhoNot)
        % Task 2  (Graduate students 5pts, Undergrads, 5pts E.C.):  
        %computes the forces that repel a point on the link that is closest to any workspace obstacle
        %at positon pObstacle, given by equation 5.6 & 5.7 in RD&C
        %q: configuration of the arm
        %pObstacle: xy position of the point obstacle
        %eta: vector parameter that scales the forces for each degree-of-freedom
        %rhoNot: defines the distance of influence of the obstacle
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

    function updateArm(o,hline,hpts)
        set(hline, 'xdata',[0;o(:,1)], 'ydata',[0;o(:,2)]);
        set(hpts, 'xdata',[0;o(:,1)], 'ydata',[0;o(:,2)]);
    end

    function d = dist(a,b)
        d=sum((a-b).^2).^.5;
    end
end