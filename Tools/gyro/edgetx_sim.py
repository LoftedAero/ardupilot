#!/usr/bin/env python3
"""
AP_FLAKE8_CLEAN

Run a radio-side EdgeTX Lua tool script (such as the ExpressLRS script,
elrsV3.lua) on the PC against SITL.

The script runs unmodified in Python's Lua engine (lupa). The parts of the
EdgeTX Lua API it uses are provided here:

- crossfireTelemetryPush/Pop talk CRSF to SITL through crsf_menu_client
- lcd.* drawing is recorded and can be rendered as text or as a PNG
- getTime, getVersion, model.getModule, popupConfirmation, bit32
- a fake transmitter module (address 0xEE) answers device pings, as the
  radio's own ELRS module would, so the script gets past "Loading..."

Keys are sent as EdgeTX virtual events: next, prev, enter, exit.

This is an approximation of EdgeTX: layout and colours are close, not
exact, and radio-link timing is not simulated. Final checks still belong
on a real radio.
"""
import os
import time

from lupa import LuaRuntime

import crsf_menu_client as cm

EVENTS = {"enter": 1, "exit": 2, "next": 3, "prev": 4}

# EdgeTX drawing flags (any distinct bits work: the script only combines them)
F_INVERS = 0x01
F_BOLD = 0x02
F_BLINK = 0x04
F_RIGHT = 0x08
F_MIDSIZE = 0x10
F_CUSTOM_COLOR = 0x20
F_DISABLED = 0x40

CHAR_W = 9          # pixels per character as reported by lcd.sizeText
TEXT_H = 20         # text line height for the colour layout

ADDR_TX_MODULE = 0xEE
# entries reported by the fake TX module. A real ELRS module has about
# this many; the script keeps its cursor line when Back switches from a
# device to the module, and with fewer module entries than that line
# its next Up or Down never finds a line (an endless loop). Only the
# first entry is shown; the rest are hidden.
TX_MODULE_FIELDS = 24


class EdgeTxSim:
    def __init__(self, script_path, client, width=480, height=272, tx_name="Sim TX"):
        self.client = client
        self.width = width
        self.height = height
        self.tx_name = tx_name
        self.ops = []               # drawing since the last lcd.clear()
        self.popup = None           # (title, message) shown this frame
        self._popup_this_frame = False
        self.local_frames = []      # frames from the fake TX module
        self.custom_color = (0, 0, 0)
        # set EDGETX_SIM_LOG to a file path to log parameter traffic
        self.log_path = os.environ.get("EDGETX_SIM_LOG")
        self.lua = LuaRuntime(unpack_returned_tuples=True)
        self._install_api()
        with open(script_path) as f:
            self.script = self.lua.execute(f.read())
        self.script.init()
        # a radio script that loops forever would hang the caller: stop
        # any single run after far more instructions than a run needs
        self._run = self.lua.eval("""function(run)
  return function(event)
    debug.sethook(function() error("script stuck in a loop") end, "", 50000000)
    local ok, result = pcall(run, event)
    debug.sethook()
    if not ok then error(result, 0) end
    return result
  end
end""")(self.script.run)

    # ----- EdgeTX API ------------------------------------------------------
    def _install_api(self):
        g = self.lua.globals()
        g.py_draw = self._draw
        g.py_clear = self._clear
        g.py_set_color = self._set_color
        g.py_push = self._push
        g.py_pop = self._pop
        g.py_popup = self._popup
        g.py_time = lambda: int(time.time() * 100)
        self.lua.execute("""
bit32 = {
  band = function(a, b) return a & b end,
  bor = function(a, b) return a | b end,
  bxor = function(a, b) return a ~ b end,
  lshift = function(a, n) return (a << n) & 0xFFFFFFFF end,
  rshift = function(a, n) return (a & 0xFFFFFFFF) >> n end,
  btest = function(a, b) return (a & b) ~= 0 end,
}
INVERS, BOLD, BLINK, RIGHT, MIDSIZE, CUSTOM_COLOR, COLOR_THEME_DISABLED =
  0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40
BLACK, WHITE, SOLID, SMLSIZE = 0x000000, 0xFFFFFF, 0, 0
EVT_VIRTUAL_ENTER, EVT_VIRTUAL_EXIT, EVT_VIRTUAL_NEXT, EVT_VIRTUAL_PREV, EVT_TOUCH_TAP = 1, 2, 3, 4, 5
LCD_W, LCD_H = %d, %d
lcd = {
  clear = function() py_clear() end,
  RGB = function(r, g, b) return (r << 16) | (g << 8) | b end,
  setColor = function(idx, c) py_set_color(c) end,
  sizeText = function(t) return %d * #tostring(t or ""), %d end,
  getLastPos = function() return 0 end,
  drawText = function(x, y, t, f) py_draw("text", x, y, tostring(t), f or 0) end,
  drawFilledRectangle = function(x, y, w, h, f) py_draw("fill", x, y, w, h, f or 0) end,
  drawRectangle = function(x, y, w, h, f) py_draw("rect", x, y, w, h, f or 0) end,
  drawLine = function(x1, y1, x2, y2, p, f) py_draw("line", x1, y1, x2, y2, f or 0) end,
  drawGauge = function(x, y, w, h, v, m, f) py_draw("rect", x, y, w, h, f or 0) end,
}
getTime = function() return py_time() end
getVersion = function() return "2.10.0", "pc-sim", 2, 10, 0, "EdgeTX" end
model = { getModule = function(i) return { Type = 5 } end }
crossfireTelemetryPush = function(cmd, data)
  local t = {}
  for i = 1, #data do t[i] = data[i] end
  return py_push(cmd, t)
end
crossfireTelemetryPop = function()
  local cmd, data = py_pop()
  if cmd == nil then return nil end
  local t = {}
  for i = 1, #data do t[i] = data[i] end
  return cmd, t
end
popupConfirmation = function(title, message, event)
  return py_popup(title, message, event)
end
""" % (self.width, self.height, CHAR_W, TEXT_H))

    def _clear(self):
        self.ops = []

    def _set_color(self, rgb):
        rgb = int(rgb)
        self.custom_color = ((rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF)

    def _draw(self, kind, *args):
        self.ops.append((kind, args, self.custom_color))

    def _log(self, direction, cmd, payload):
        if self.log_path and cmd in (cm.T_READ, cm.T_WRITE, cm.T_ENTRY, cm.T_DEVICE_INFO, cm.T_PING):
            with open(self.log_path, "a") as f:
                f.write("%.3f %s 0x%02x %s\n" % (time.time(), direction, cmd, list(payload)[:8]))

    def _push(self, cmd, data):
        payload = [int(data[i]) for i in range(1, len(data) + 1)]
        self._log("TX", cmd, payload)
        if cmd == cm.T_PING:
            # the radio's own ELRS module answers a ping. Fake one with info
            # entries: the script only sets up its entry list for a device
            # that reports at least one entry
            name = self.tx_name.encode() + b"\0"
            info = bytes([0xEA, ADDR_TX_MODULE]) + name + b"SIMU" + bytes(8) + bytes([TX_MODULE_FIELDS, 0])
            self.local_frames.append((cm.T_DEVICE_INFO, info))
        if len(payload) > 0 and payload[0] == ADDR_TX_MODULE:
            if cmd == cm.T_READ and len(payload) >= 3 and 1 <= payload[2] <= TX_MODULE_FIELDS:
                field_id = payload[2]
                if field_id == 1:
                    entry = bytes([0, 12]) + b"Module\0PC simulator\0"
                else:
                    entry = bytes([0, 12 | 0x80]) + b"Spare\0-\0"     # hidden
                self.local_frames.append((cm.T_ENTRY, bytes([0xEA, ADDR_TX_MODULE, field_id, 0]) + entry))
            return True
        self.client.send(cmd, payload)
        return True

    def _pop(self):
        if self.local_frames:
            ftype, payload = self.local_frames.pop(0)
        else:
            frame = self.client.pop_frame()
            if frame is None:
                return None
            ftype, payload = frame
        self._log("RX", ftype, payload)
        return ftype, self.lua.table_from(list(payload))

    def _popup(self, title, message, event):
        self.popup = (str(title), str(message))
        self._popup_this_frame = True
        if event == EVENTS["enter"]:
            return "OK"
        if event == EVENTS["exit"]:
            return "CANCEL"
        return None

    # ----- driving the script ---------------------------------------------
    def step(self, key=None):
        """run the script once; returns its result (non-zero: the script
        exited). Raises lupa.LuaError if the script fails or gets stuck"""
        self._popup_this_frame = False
        result = self._run(EVENTS[key] if key else 0)
        if not self._popup_this_frame:
            self.popup = None
        return result

    def idle(self, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            self.step()
            time.sleep(0.02)

    def press(self, key, settle=0.4):
        self.step(key)
        self.idle(settle)

    # ----- reading the screen ---------------------------------------------
    def lines(self):
        """screen text as rows of (text, selected) cells, top to bottom"""
        rows = {}
        for kind, args, _ in self.ops:
            if kind != "text":
                continue
            x, y, text, flags = args
            if not text:
                continue
            if flags & F_RIGHT:
                x -= CHAR_W * len(text)
            rows.setdefault(int(round(y)), []).append((int(x), text, bool(flags & F_INVERS)))
        return [sorted(cells) for _, cells in sorted(rows.items())]

    def text(self):
        out = []
        for cells in self.lines():
            line = ""
            for x, text, selected in cells:
                col = max(0, x // CHAR_W)
                if len(line) < col:
                    line += " " * (col - len(line))
                elif line:
                    line += " "
                line += ("\u00ab" + text + "\u00bb") if selected else text
            out.append(line.rstrip())
        if self.popup:
            out.append("")
            out.append("+--- popup " + "-" * 30)
            out.append("| " + self.popup[0])
            out.append("| " + self.popup[1])
            out.append("+" + "-" * 41)
        return "\n".join(out)

    def selected(self):
        for cells in self.lines():
            for _, text, sel in cells:
                if sel:
                    return text
        return None

    def contains(self, text):
        return any(text in t for cells in self.lines() for _, t, _ in cells)

    def select(self, label, max_steps=40):
        """move the cursor until the selected entry reads label (folders
        and commands are drawn with decorations, so match by containment)"""
        for key in ("next", "prev"):
            for _ in range(max_steps):
                sel = self.selected()
                if sel is not None and label in sel:
                    return True
                before = sel
                self.press(key, settle=0.15)
                if self.selected() == before:
                    break       # reached the end of the list
        return False

    def wait_for(self, text, timeout=20):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.idle(0.2)
            if self.contains(text) or (self.popup and any(text in p for p in self.popup)):
                return True
        return False

    def save_png(self, path):
        """render the screen; path is a file name or a binary file object"""
        from PIL import Image, ImageDraw, ImageFont
        img = Image.new("RGB", (self.width, self.height), (255, 255, 255))
        d = ImageDraw.Draw(img)
        font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
        bold_path = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf"
        font = ImageFont.truetype(font_path, 15) if os.path.exists(font_path) else ImageFont.load_default()
        bold = ImageFont.truetype(bold_path, 15) if os.path.exists(bold_path) else font
        for kind, args, color in self.ops:
            if kind == "fill":
                x, y, w, h, _ = args
                if w > 0 and h > 0:
                    d.rectangle([x, y, x + w - 1, y + h - 1], fill=color)
            elif kind == "rect":
                x, y, w, h, _ = args
                d.rectangle([x, y, x + w - 1, y + h - 1], outline=color)
            elif kind == "line":
                x1, y1, x2, y2, _ = args
                d.line([x1, y1, x2, y2], fill=color)
            elif kind == "text":
                x, y, text, flags = args
                if not text:
                    continue
                f = bold if flags & F_BOLD else font
                if flags & F_RIGHT:
                    x -= f.getsize(text)[0]
                fg = (128, 128, 128) if flags & F_DISABLED else (0, 0, 0)
                if flags & F_INVERS:
                    w, h = f.getsize(text)
                    d.rectangle([x - 1, y, x + w + 1, y + h + 2], fill=(0, 0, 0))
                    fg = (255, 255, 255)
                d.text((x, y), text, fill=fg, font=f)
        if self.popup:
            box = [30, 70, self.width - 30, self.height - 70]
            d.rectangle(box, fill=(240, 240, 240), outline=(0, 0, 0))
            d.text((box[0] + 12, box[1] + 18), self.popup[0], fill=(0, 0, 0), font=bold)
            d.text((box[0] + 12, box[1] + 48), self.popup[1], fill=(0, 0, 0), font=font)
        img.save(path, format="PNG")
