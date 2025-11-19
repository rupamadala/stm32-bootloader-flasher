import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext, simpledialog, ttk
import serial
import serial.tools.list_ports
import threading
import time
import os


# ============================================================
#  Utility functions (Thread-Safe GUI wrappers)
# ============================================================

def gui_log(msg):
    """Safe log output from threads."""
    window.after(0, lambda: (log_box.insert(tk.END, msg + "\n"), log_box.see(tk.END)))


def gui_error(msg):
    window.after(0, lambda: messagebox.showerror("Error", msg))


def gui_progress(val):
    window.after(0, lambda: progress_bar.config(value=val))


def gui_set_max_progress(val):
    window.after(0, lambda: progress_bar.config(maximum=val))


# ============================================================
#  Intel HEX Parsing
# ============================================================

def parse_hex_line(line):
    """Parse an Intel HEX record. Returns address, data, rectype."""
    line = line.strip()
    if not line or line[0] != ":":
        return None, None, None, None

    line = line[1:]  # remove ':'

    length = int(line[0:2], 16)
    address = int(line[2:6], 16)
    rectype = int(line[6:8], 16)
    data = bytes.fromhex(line[8:8 + length * 2])
    checksum = int(line[8 + length * 2:8 + length * 2 + 2], 16)

    return address, data, checksum, rectype


# ============================================================
#  CRC Calculation (SUM of all bytes mod 256)
# ============================================================

def calc_crc(frame_wo_len):
    return sum(frame_wo_len) & 0xFF


# ============================================================
#  UART Frame Sending
# ============================================================

def send_hex_frame(ser, frame):
    """Send frame byte-by-byte."""
    for b in frame:
        ser.write(bytes([b]))
        time.sleep(0.001)

    frame_str = ' '.join(f"{b:02X}" for b in frame)
    gui_log(f"➡️ Sent HEX frame: {frame_str}")


def wait_for_ack(ser, max_wait_ms=2000):
    deadline = time.time() + max_wait_ms / 1000.0

    while time.time() < deadline:
        resp = ser.read(1)
        if resp:
            if resp[0] == 0x79:
                gui_log("✅ ACK (0x79)")
                return True
            elif resp[0] == 0x81:
                gui_log("❌ NACK (0x81)")
                return False
        time.sleep(0.005)

    gui_log("❌ No ACK received")
    return False

# ============================================================
#  ERASE memory
# ============================================================

def send_erase():
    """Ask user for start & end address, then launch thread."""
    start_str = simpledialog.askstring("Start Address", "Enter start address (hex):")
    end_str = simpledialog.askstring("End Address", "Enter end address (hex):")

    if not start_str or not end_str:
        return

    try:
        start_addr = int(start_str, 16)
        end_addr = int(end_str, 16)
    except ValueError:
        gui_error("Invalid address input.")
        return

    threading.Thread(target=_erase_thread,
                     args=(start_addr, end_addr),
                     daemon=True).start()


def _erase_thread(start_addr, end_addr):
    port = port_var.get()
    baud = int(baud_var.get())

    try:
        ser = serial.Serial(port, baud, timeout=2)
    except Exception as e:
        gui_error(str(e))
        return

    try:
        cmd = 0x00

        # Build frame: CMD + StartAddr(4) + EndAddr(4)
        frame_wo_len = bytearray([cmd])
        frame_wo_len += start_addr.to_bytes(4, "big")
        frame_wo_len += end_addr.to_bytes(4, "big")

        crc = calc_crc(frame_wo_len)

        # LEN = all bytes except the LEN itself
        frame = bytearray([len(frame_wo_len) + 1]) + frame_wo_len + bytes([crc])

        gui_log(f"📤 Sending ERASE from 0x{start_addr:08X} to 0x{end_addr:08X}")
        send_hex_frame(ser, frame)

        if not wait_for_ack(ser, max_wait_ms=12000):
            gui_log("❌ ERASE failed: No ACK")
            return

        gui_log("✅ ERASE completed")

    finally:
        ser.close()


# ============================================================
#  GO TO address
# ============================================================

def send_go_to_address():
    """Prompt address on main thread, then start UART op."""
    addr_str = simpledialog.askstring("Enter Address", "Enter 32-bit address (e.g. 08000000):")
    if not addr_str:
        return

    try:
        address = int(addr_str, 16)
    except ValueError:
        gui_error("Invalid address.")
        return

    threading.Thread(target=_go_thread, args=(address,), daemon=True).start()


def _go_thread(address):
    port = port_var.get()
    baud = int(baud_var.get())

    try:
        ser = serial.Serial(port, baud, timeout=3)
    except Exception as e:
        gui_error(str(e))
        return

    try:
        cmd = 0x01
        frame_wo_len = bytearray([cmd]) + address.to_bytes(4, 'big')  # Big Endian
        crc = calc_crc(frame_wo_len)
        frame = bytearray([len(frame_wo_len) + 1]) + frame_wo_len + bytes([crc])

        send_hex_frame(ser, frame)
        wait_for_ack(ser)

    finally:
        ser.close()


# ============================================================
#  READ FLASH
# ============================================================

def send_flash_read():
    addr_str = simpledialog.askstring("Enter Address", "Enter 32-bit address to read:")
    if not addr_str:
        return

    try:
        address = int(addr_str, 16)
    except ValueError:
        gui_error("Invalid address.")
        return

    threading.Thread(target=_read_thread, args=(address,), daemon=True).start()


def _read_thread(address):
    port = port_var.get()
    baud = int(baud_var.get())

    try:
        ser = serial.Serial(port, baud, timeout=3)
    except Exception as e:
        gui_error(str(e))
        return

    try:
        cmd = 0x03
        frame_wo_len = bytearray([cmd]) + address.to_bytes(4, 'big')
        crc = calc_crc(frame_wo_len)
        frame = bytearray([len(frame_wo_len) + 1]) + frame_wo_len + bytes([crc])

        gui_log(f"📤 Reading flash at 0x{address:08X}")
        send_hex_frame(ser, frame)

        # MCU sends 8 bytes of data, then ACK
        data = ser.read(8)
        if len(data) == 8:
            data_str = ' '.join(f"{b:02X}" for b in data)
            gui_log(f"📖 Data: {data_str}")
        else:
            gui_log("⚠️ Incomplete data")

        wait_for_ack(ser)

    finally:
        ser.close()


# ============================================================
#  SEND HEX FILE
# ============================================================

def send_hex_file():
    threading.Thread(target=_send_hex_file_thread, daemon=True).start()


def _send_hex_file_thread():
    port = port_var.get()
    baud = int(baud_var.get())
    filepath = file_path_var.get()

    if not filepath:
        gui_error("Select a HEX file.")
        return

    try:
        ser = serial.Serial(port, baud, timeout=3)
    except Exception as e:
        gui_error(str(e))
        return

    try:
        with open(filepath, "r") as f:
            lines = f.readlines()

        gui_log(f"\n📁 Sending HEX: {os.path.basename(filepath)}")
        gui_set_max_progress(len(lines))
        gui_progress(0)

        base_addr = 0
        index = 0

        for line in lines:
            addr, data, _, rectype = parse_hex_line(line)
            if addr is None:
                continue

            # Extended Linear Address
            if rectype == 0x04:
                base_addr = int(line[9:13], 16) << 16
                continue

            # End Of File
            if rectype == 0x01:
                break

            if rectype != 0x00:
                continue

            full_address = base_addr + addr
            cmd = 0x02  # WRITE

            frame_wo_len = bytearray([cmd]) + full_address.to_bytes(4, 'big') + data
            crc = calc_crc(frame_wo_len)

            frame = bytearray([len(frame_wo_len) + 1]) + frame_wo_len + bytes([crc])

            send_hex_frame(ser, frame)

            # Require ACK before next frame
            if not wait_for_ack(ser):
                gui_log(f"❌ Stopped at address 0x{full_address:08X}")
                break

            index += 1
            gui_progress(index)
            time.sleep(0.02)

        gui_log("✅ HEX transmission complete.")
        # After finishing all hex lines
        cmd = 0x04   # END FRAME command
        
        frame_wo_len = bytearray([cmd])
        crc = calc_crc(frame_wo_len)
        frame = bytearray([len(frame_wo_len) + 1]) + frame_wo_len + bytes([crc])
        
        gui_log("📤 Sending END-FRAME command…")
        send_hex_frame(ser, frame)
        wait_for_ack(ser)


    except Exception as e:
        gui_error(str(e))
    finally:
        ser.close()


# ============================================================
#  GUI SETUP
# ============================================================

window = tk.Tk()
window.title("HEX UART Sender")
window.geometry("800x650")

tk.Label(window, text="HEX File:").grid(row=0, column=0, padx=5, pady=5)
file_path_var = tk.StringVar()
tk.Entry(window, textvariable=file_path_var, width=60).grid(row=0, column=1)
tk.Button(window, text="Browse", command=lambda: file_path_var.set(
    filedialog.askopenfilename(filetypes=[("HEX Files", "*.hex")])
)).grid(row=0, column=2)

tk.Label(window, text="COM Port:").grid(row=1, column=0)
port_var = tk.StringVar()
port_menu = tk.OptionMenu(window, port_var, "")
port_menu.grid(row=1, column=1, sticky="w")

tk.Button(window, text="Refresh", command=lambda: refresh_ports()).grid(row=1, column=2)

tk.Label(window, text="Baud Rate:").grid(row=2, column=0)
baud_var = tk.StringVar(value="115200")
tk.Entry(window, textvariable=baud_var, width=10).grid(row=2, column=1, sticky="w")

button_frame = tk.Frame(window)
button_frame.grid(row=3, column=0, columnspan=3, pady=10)

tk.Button(button_frame, text="Erase", width=15, bg="#E53935", fg="white",
          command=send_erase).grid(row=0, column=0, padx=10)

tk.Button(button_frame, text="Go To Addr", width=15, bg="#039BE5", fg="white",
          command=send_go_to_address).grid(row=0, column=1, padx=10)

tk.Button(button_frame, text="Send HEX", width=15, bg="#43A047", fg="white",
          command=send_hex_file).grid(row=0, column=2, padx=10)

tk.Button(button_frame, text="Read Flash", width=15, bg="#FB8C00", fg="white",
          command=send_flash_read).grid(row=1, column=1, pady=5)


progress_bar = ttk.Progressbar(window, length=700)
progress_bar.grid(row=4, column=0, columnspan=3, pady=10)

log_box = scrolledtext.ScrolledText(window, width=95, height=20)
log_box.grid(row=5, column=0, columnspan=3, padx=10, pady=10)


def refresh_ports():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    menu = port_menu["menu"]
    menu.delete(0, "end")
    for p in ports:
        menu.add_command(label=p, command=lambda v=p: port_var.set(v))
    if ports:
        port_var.set(ports[0])


refresh_ports()
window.mainloop()
