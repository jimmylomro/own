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
        '-lopencv_features2d -lopencv_imgproc ' ...
	    '-lopencv_calib3d' ' -lopencv_highgui ' ...
        '-lopencv_core ' '-lopencv_imgproc ' '-lopencv_flann ']); %...
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
