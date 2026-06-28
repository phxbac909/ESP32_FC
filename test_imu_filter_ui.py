import socket
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import collections
import datetime

# --- CẤU HÌNH ---
ESP32_IP = "192.168.100.207" # <--- SỬA LẠI IP CỦA BẠN TẠI ĐÂY
ESP32_PORT = 8888

# --- BIẾN TOÀN CỤC ---
data_length = 400 # Lưu được 20 giây data (ở tốc độ 20Hz telemetry)
time_data = collections.deque([0]*data_length, maxlen=data_length)
pitch_data = collections.deque([0.0]*data_length, maxlen=data_length)
roll_data = collections.deque([0.0]*data_length, maxlen=data_length)
throttle_data = collections.deque([1000]*data_length, maxlen=data_length)
acc_mag_data = collections.deque([0.0]*data_length, maxlen=data_length)

counter = 0
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", ESP32_PORT))

# ==========================================
# CÁC HÀM GIAO TIẾP MẠNG
# ==========================================
def receive_telemetry():
    global counter
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            msg = data.decode('utf-8').strip()
            
            if msg.startswith("D,"):
                parts = msg.split(',')
                if len(parts) >= 5:          # đổi 4 → 5
                    thr   = int(parts[1])
                    pitch = float(parts[2])
                    roll  = float(parts[3])
                    amag  = float(parts[4])  # thêm dòng này
                    
                    time_data.append(counter)
                    throttle_data.append(thr)
                    pitch_data.append(pitch)
                    roll_data.append(roll)
                    acc_mag_data.append(amag) # thêm dòng này
                    counter += 1
        except Exception as e:
            pass

def send_throttle(val=None):
    thr = int(float(throttle_slider.get()))
    msg = f"T{thr}"
    sock.sendto(msg.encode(), (ESP32_IP, ESP32_PORT))

# HEARTBEAT: Gửi ga liên tục mỗi 100ms để ESP32 không bị Failsafe
def keep_alive():
    send_throttle()
    root.after(100, keep_alive)

def kill_motors():
    throttle_slider.set(1000)
    send_throttle()

# ==========================================
# HÀM XUẤT FILE MARKDOWN (CHO AI PHÂN TÍCH)
# ==========================================
def export_to_md():
    now = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"Telemetry_Export_{now}.md"
    
    try:
        with open(filename, "w", encoding="utf-8") as f:
            f.write(f"# Drone Telemetry Data\n")
            f.write(f"**Date:** {now}\n\n")
            f.write("Dữ liệu test rung động động cơ.\n\n")
            f.write("```csv\n")
            f.write("Sample,Throttle_PWM,Pitch_Deg,Roll_Deg,AccMag_G\n") # thêm cột
            
            t_list   = list(time_data)
            thr_list = list(throttle_data)
            p_list   = list(pitch_data)
            r_list   = list(roll_data)
            a_list   = list(acc_mag_data)  # thêm dòng này
            
            for i in range(len(t_list)):
                if t_list[i] != 0 or i == 0:
                    f.write(f"{t_list[i]},{thr_list[i]},{p_list[i]:.2f},{r_list[i]:.2f},{a_list[i]:.3f}\n") # thêm cột
            
            f.write("```\n")
            
        messagebox.showinfo("Thành công", f"Đã xuất:\n{filename}")
    except Exception as e:
        messagebox.showerror("Lỗi", f"Không thể xuất file: {e}")

# ==========================================
# GIAO DIỆN TKINTER & MATPLOTLIB
# ==========================================
root = tk.Tk()
root.title("ESP32 Drone Telemetry")
root.geometry("1000x700")

# --- FRAME ĐIỀU KHIỂN BÊN TRÁI ---
control_frame = tk.Frame(root, width=200)
control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=20, pady=20)

tk.Label(control_frame, text="THROTTLE", font=('Arial', 14, 'bold')).pack(pady=10)

throttle_slider = ttk.Scale(control_frame, from_=2000, to=1000, orient=tk.VERTICAL, length=300, command=send_throttle)
throttle_slider.set(1000)
throttle_slider.pack(pady=10)

kill_btn = tk.Button(control_frame, text="KILL", bg="red", fg="white", font=('Arial', 14, 'bold'), command=kill_motors)
kill_btn.pack(pady=20, fill=tk.X)

export_btn = tk.Button(control_frame, text="EXPORT TO .MD", bg="green", fg="white", font=('Arial', 12, 'bold'), command=export_to_md)
export_btn.pack(pady=10, fill=tk.X)

# --- FRAME ĐỒ THỊ BÊN PHẢI ---
plot_frame = tk.Frame(root)
plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

# Tạo 3 đồ thị xếp dọc dùng chung trục X (sharex=True)
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
fig.tight_layout(pad=3.0)

# Vẽ các đường Line ban đầu
line_thr, = ax1.plot(time_data, throttle_data, 'r-', label='Throttle (PWM)')
line_pitch, = ax2.plot(time_data, pitch_data, 'b-', label='Pitch (Deg)')
line_roll, = ax3.plot(time_data, roll_data, 'g-', label='Roll (Deg)')

# Set Limits và Labels
ax1.set_ylim(950, 2050)
ax1.set_ylabel('PWM', color='r', fontweight='bold')
ax1.legend(loc="upper left")
ax1.grid(True, linestyle='--', alpha=0.6)

ax2.set_ylim(-30, 30)
ax2.set_ylabel('Pitch', color='b', fontweight='bold')
ax2.legend(loc="upper left")
ax2.grid(True, linestyle='--', alpha=0.6)

ax3.set_ylim(-30, 30)
ax3.set_ylabel('Roll', color='g', fontweight='bold')
ax3.set_xlabel('Samples')
ax3.legend(loc="upper left")
ax3.grid(True, linestyle='--', alpha=0.6)

canvas = FigureCanvasTkAgg(fig, master=plot_frame)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

# Hàm update đồ thị liên tục
def update_plot():
    if len(time_data) > 0:
        # Cập nhật data cho 3 đường
        line_thr.set_data(time_data, throttle_data)
        line_pitch.set_data(time_data, pitch_data)
        line_roll.set_data(time_data, roll_data)
        
        # Dịch trục X chạy theo
        ax3.set_xlim(time_data[0], time_data[-1] + 1)
        
        canvas.draw()
    
    root.after(50, update_plot) 

# ==========================================
# KHỞI CHẠY CÁC LUỒNG
# ==========================================
threading.Thread(target=receive_telemetry, daemon=True).start()
root.after(100, keep_alive) # Bắt đầu nhịp tim gửi UDP
root.after(50, update_plot) # Bắt đầu vẽ đồ thị

root.mainloop()