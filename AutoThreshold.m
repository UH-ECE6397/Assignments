function zt = AutoThreshold(img)
% Given a gray-scale image img, finds a threshold value zt that minimizes
% the within-group variance, giving a threshold that divides the image into
% two groups.
% Uses the algorithm in "Robot Modeling and Control" by Spong, Hutchinson,
% and Vidyasagar
%
% Aaron T Becker, 03-21-2016, atbecker@uh.edu
% http://www.labbookpages.co.uk/software/imgProc/otsuThreshold.html
%
%  Items to complete are marked with "TODO:"
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%load image
if nargin <1  % if user doesn't specify an image, load one from memory
    img = imread('Duplos.png');
    %img = imread('coins.png');
end

% convert image to black and white (if color)
if numel(size(img))>2
    Image = rgb2gray(img);
else
    Image = img;
end
N = max(Image(:))+1; %total number of gray levels

%compute the histogram (11.3.1, page 386)
H = zeros(N,1);
[Nrows,Ncols] = size(Image); 
for r = 1:Nrows
    for c = 1:Ncols
        index = Image(r,c)+1; %matlab indices count from 0, so add 1
        H(index) = H(index)+1; %increment count in bin
    end
end

% compute the probability of each grey level (11.10)
P = H/(Nrows*Ncols);

% compute the mean (11.11)
mu =  50; %TODO #1
% compute the image variance (11.13)
sigSq =  500; %TODO #2

%allocate variables
q0 = zeros(N,1);
mu0 = zeros(N,1);
mu1 = zeros(N,1);
sigb2 = zeros(N,1);

%initialize variables
q0(1) = P(1);
mu0(1)  = 0; %always zero
mu1(1) =  mu;
for zt = 1:N-1
    % compute q0 (11.17)   (q0 = cumsum(P))
    q0(zt+1) = 50; %TODO #2;
    
    % compute mu0 (11.18)
    if q0(zt+1)>0  %mu0f = cumsum(zP)./q0f;
        mu0(zt+1)   =  20; %TODO #3
    else  % avoid divide by zero error
        mu0(zt+1)   = 0;
    end
    
    %compute mu1 (11.19)
    if q0(zt+1) < 1
        mu1(zt+1) =  20; %TODO #4
    else  % avoid divide by zero error
        mu1(zt+1) = zt+1;
    end
    
    % compute sigb2 (11.16), the between-group variance
    sigb2(zt+1) = 20; %TODO #5
    
end


[~, zt] = max(sigb2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% generate the plots.
figure(1); clf

subplot(2,2,1)
imshow(Image)
title(['image with ',num2str(N),' gray levels'])

subplot(2,2,2)
imshow(Image>zt)
title(['thresholded at z>',num2str(zt)])

subplot(2,2,3)
%bar(0:N-1,H); axis tight
imhist(Image)
axis tight
title('histogram of image')
xlabel({'';'';'grayscale values'})
ylabel('counts')
line([zt,zt],[0,max(H)],'Color','r')

subplot(2,2,4)
plot(0:N-1,sigSq-sigb2')
axis tight
line([zt,zt],[0,sigSq],'Color','r')
title('within-group variance of image')
ylabel('variance')
xlabel('threshold values')

%%%%%Figure 2%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(2); clf
subplot(2,2,1)
plot(1:N,mu0,1:N,mu1,[0,N],[mu,mu],'r') 
axis([0,N,0,N])
legend('\mu_0','\mu_1','\mu','location','best')
xlabel('gray value')
ylabel('mean')

subplot(2,2,2)
plot(0:N-1,sigb2')
line([zt,zt],[0,sigSq],'Color','r')
axis([0,double(N),0,sigSq])
title('between-group variance of image')
ylabel('variance')
xlabel('threshold values')

subplot(2,2,3)
plot(1:N,q0,1:N,1-q0,[0,N],[1,1],'r') 
axis([0,N,0,1.1])
legend('q_0','q_1','location','best')
title('between-group variance of image')
ylabel('probability')
xlabel('threshold values')

subplot(2,2,4)
bar(0:N-1,H); axis tight
axis tight
title('histogram of image')
xlabel({'grayscale values'})
ylabel('counts')
line([zt,zt],[0,max(H)],'Color','r')
