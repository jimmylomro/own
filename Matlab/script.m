%
% Usage example
%

%%
% ------------------------------- Initialise parameters
K 			= 64;
kernSize	= 24;


% ------------------------------- Initialise interface
own_interface('init','threshold',0.3,'kernSize',kernSize,'nMaps',K);


% ------------------------------- Extract Keypoints from images
im = imread('../res/im.jpg');    
im = imresize(im,0.5);

own_interface('loadImage',im);

keypoints = own_interface('detect');
    
own_interface('terminate');

%%
% ------------------------------- Show the images with the keypoints
for i = 1:K
    
    idx = keypoints(:,5) == i-1;
    
    figure();
    imshow(im);
    hold on;
    plot(keypoints(idx,1),keypoints(idx,2),'b*');
    hold off;
    
end
