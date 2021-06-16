"""
    Process killer...
"""
import win32api


def stop_process(pid):
    handle = win32api.OpenProcess(1, False, pid)  # 获取进程句柄（pid类型为int！）
    win32api.TerminateProcess(handle, 0)  # 杀了它
    win32api.CloseHandle(handle)  # 关闭
    print("已杀死进程(pid={pid})".format(pid=pid))
