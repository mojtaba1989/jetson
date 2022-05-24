import sys

gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')

if __name__ == "__main__":
    GObject.threads_init()
loop = GObject.MainLoop()
Gst.init(None)

server = GstRtspServer.RTSPServer()
server.set_service('554')

factory = GstRtspServer.RTSPMediaFactory()
factory.set_launch("( rpicamsrc name=src bitrate=0 ",
                   "quantisation-parameter=24 intra-refresh-type=cyclic",
                   "metering-mode=matrix annotation-mode=12 preview=false !",
                   "video/x-h264,profile=baseline,width=640,height=480,framerate=30/1 !",
                   "h264parse ! rtph264pay pt=96 name=pay0 )")
factory.set_shared(True)

server.get_mount_points().add_factory("/video", factory)

server.attach(None)
print("stream ready at rtsp://paitu:554/video")

loop.run()