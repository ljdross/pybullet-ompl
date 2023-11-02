import time
import obsws_python as obs


def start_recording():
    cl = obs.ReqClient()
    cl.start_record()


def stop_recording():
    cl = obs.ReqClient()
    cl.stop_record()
    time.sleep(6)

