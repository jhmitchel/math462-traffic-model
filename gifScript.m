nImages = length(vidmat);

figure;
for idx = 1:nImages
    imshow(vidmat(:,:,idx));
    frame = getframe(1);
    im{idx} = frame2im(frame);
    
end

filename = 'magical.gif'; % Specify the output file name
for idx = 1:nImages
    [A,map] = rgb2ind(im{idx},256);
    if idx == 1
        imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',0);
    else
        imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',0);
    end
end