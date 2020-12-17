img = imread('houses.png');
if ndims(img) == 3
    img = rgb2gray(img);
end
img = double(img);

img_constrast = img.*1.5;

imshow(img_constrast,[],'InitialMagnification','fit')
title('Et gråtonebilde med økt kontrast?')

figure() % For å lage et nytt vindu
imshow(img_constrast,[0 255],'InitialMagnification','fit')
title('Et gråtonebilde med økt kontrast')
