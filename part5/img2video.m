% Image to Video Converter
framesPath = '..\part4\optimized\';%图像序列所在路径，同时要保证图像大小相同
videoName = 'optimized.avi';%表示将要创建的视频文件的名字
fps = 16; %帧率
startFrame = 0; %从哪一帧开始
endFrame = 69; %哪一帧结束

if(exist('videoName','file'))
    delete videoName.avi
end

%生成视频的参数设定
aviobj=VideoWriter(videoName);  %创建一个avi视频文件对象，开始时其为空
aviobj.FrameRate=fps;

open(aviobj);%Open file for writing video data
%读入图片
for i=startFrame:endFrame
    fileName=sprintf('%04d',i);    %根据文件名而定 我这里文件名是0001.jpg 0002.jpg ....
    frames=imread([framesPath,fileName,'.jpg']);
    writeVideo(aviobj,frames);
end
close(aviobj);% 关闭创建视频