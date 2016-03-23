function cc = ConnectedComponents( binary_img )
% Using the same image, label the connected components using the two-pass
% algorithm from section 11.4.  Call your file  ConnectedComponents.m with
% function call
% cc = ConnectedComponents( binary_img ), where binary_img is a binary image
% and cc is a matrix the size of binary_img with 0 assigned to background
% pixels and integers to different connected components.
% Uses the algorithm in "Robot Modeling and Control" by Spong, Hutchinson,
% and Vidyasagar
%
% Aaron T Becker, 03-21-2016, atbecker@uh.edu
% http://www.labbookpages.co.uk/software/imgProc/otsuThreshold.html
%
%  Items to complete are marked with "TODO:"
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
