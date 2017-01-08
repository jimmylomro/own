% mex library build script
% written by Stefan Leutenegger, 8/2011

% mex build command
clc;
clear all;

% set the openCV path:
windowsOpenCvDir='C:/OpenCV2.2';
unixOpenCvDir='/usr/local';
appleOpenCvDir='';

% determine the build and mex directory
c='';
directory='';
switch computer
    case 'GLNXA64' 
        c='glnxa64';
        directory='unix64';
    case 'GLNX86' 
        c='glnx86';
        directory='unix32';
    case 'PCWIN' 
        c='pcwin';
        directory='win32';
    case 'PCWIN64' 
        c='pcwin64';
    case 'MACI' 
        c='maci';
        directory='apple';
    case 'MACI64' 
        c='maci64';
        directory='apple';
    otherwise
        disp('error determining architecture');
end

if strcmp(computer,'PCWIN')
    mex('own_interface.cpp', ...
    ['-I' windowsOpenCvDir '/include'], ...
	'-I../include', ...
    '-L../lib', ...
    '-lown_static -lbfm_static ', ...
    '-lopencv_features2d', '-lopencv_imgproc', ...
    '-lopencv_calib3d', '-lopencv_highgui', ...
    '-lopencv_core', '-lopencv_imgproc', '-lopencv_flann');
else
    if(strcmp(directory,'apple'))
        eval(['mex own_interface.cpp -v ' ...
        '-I' unixOpenCvDir '/include ' ...
    	'-I../include ' ...
        '-L../lib  -lown_static -lbfm_static ' ...
        '-L' unixOpenCvDir '/lib ' ...
        '-lopencv_calib3d -lopencv_core -lopencv_features2d -lopencv_flann ' ...
        '-lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_ml ' ...
        '-lopencv_objdetect -lopencv_photo -lopencv_shape -lopencv_stitching ' ...
        '-lopencv_superres -lopencv_video -lopencv_videoio -lopencv_videostab ' ...
        '-lopencv_aruco -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib ' ...
        '-lopencv_datasets -lopencv_dnn -lopencv_dpm -lopencv_face -lopencv_fuzzy ' ...
        '-lopencv_line_descriptor -lopencv_optflow -lopencv_phase_unwrapping ' ...
        '-lopencv_plot -lopencv_reg -lopencv_rgbd -lopencv_saliency -lopencv_stereo '...
        '-lopencv_structured_light -lopencv_surface_matching -lopencv_text ' ...
        '-lopencv_tracking -lopencv_xfeatures2d -lopencv_ximgproc -lopencv_xobjdetect ' ...
        '-lopencv_xphoto ' ...
        '-L' unixOpenCvDir '/share/OpenCV/3rdparty/lib ' ...
        '-llibprotobuf -llibjpeg -llibwebp -llibpng -llibtiff -llibjasper ' ...
        '-lIlmImf -lzlib -lippicv' ]); %...
        %'-Wl']);
    else
        eval(['mex own_interface.cpp -v ',...
        '-I' unixOpenCvDir '/include ' ...
    	'-I../include ' ...
        '-L../' directory '/lib ']); %...
        %'-Wl']);
    end

    % kind of dangerous, but a nice hack:
    %fix_error;

end
