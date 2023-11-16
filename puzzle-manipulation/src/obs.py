import time
import obsws_python as obs


def start_recording():
    try:
        cl = obs.ReqClient()
        cl.start_record()
    except:
        print("Cannot start recording because of no connection to OBS websocket")


def stop_recording():
    try:
        cl = obs.ReqClient()
        cl.stop_record()
        time.sleep(10)
    except:
        print("Cannot stop recording because of no connection to OBS websocket")

