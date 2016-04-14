function PotentialFieldNavigation
% A path planner for an n-link planar robot arm moving amoung polygonal obstacles.
%
%
% Based off chapter 5.2 in "Robot Modeling and Control" by Spong, Hutchinson,
% and Vidyasagar
%
% Aaron T Becker, 04-13-2016, atbecker@uh.edu
%
%  Items to complete are marked with "TODO:"
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
linkLen = [2;2;2];  %lengths of each link
q = rand(numel(linkLen),1)*2*pi;  %robot configuration
qGoal = rand(numel(linkLen),1)*2*pi; %TODO: some sort of check to make sure this isn't intersecting an obstacle
oGoal = computeOrigins(qGoal); %robot goal DH origins 

% Parameters  (students should change these)
zeta =   flipud(.1*cumsum(ones(numel(q),1)));
alpha = 0.02;
d = 0.5;

%setup figure
figure(1);clf;
r = sum(linkLen);
rectangle('Position',[-r,-r,2*r,2*r],'Curvature',1,'FaceColor',[.9 .9 1]) %robot workspace
axis equal
%draw robot
hold on
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
    Fvec =  fatt(q, oGoal,zeta,d);
    
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
        % o is a vector of the origins for DH frames of a planar robot arm.
        Fvec = zeros(numel(q),2);
        o = computeOrigins(q);
        
        
        
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

    function updateArm(o,hline,hpts)
        set(hline, 'xdata',[0;o(:,1)], 'ydata',[0;o(:,2)]);
        set(hpts, 'xdata',[0;o(:,1)], 'ydata',[0;o(:,2)]);
    end

    function d = dist(a,b)
        d=sum((a-b).^2).^.5;
    end
end