#!/usr/bin/env python

# Operator Console for Team ZARJ
#  Beginning code

import threading
import pygtk
pygtk.require("2.0")
import gtk
import gtk.glade
import sys
import os
import math
import cv2
import Queue
from copy import deepcopy

# This is required to use threads + gtk together
import gobject
gobject.threads_init()

# Okay, this is mildly lame, but it's quick + dirty
sys.path.append('./lib')
sys.path.append('../lib')
sys.path.append('../../lib')

sys.path.append('./fc')
sys.path.append('../fc')
sys.path.append('../../fc')

from zarjcomm import ZarjComm
from zarjcomm.message import ZarjMessage
from zarjcomm.message import ZarjStatus
from zarjcomm.message import ZarjSatellite
from zarjcomm.message import ZarjLeak
from zarjcomm.message import ZarjPicture
from zarjcomm.message import ZarjLaserScan
from zarjcomm.message import ZarjIntervalCommand
from zarjcomm.message import ZarjStartCommand
from zarjcomm.message import ZarjStopCommand
from zarjcomm.message import ZarjMacroCommand
from zarjcomm.message import ZarjExitCommand
from zarjcomm.message import ZarjWalkCommand
from zarjcomm.message import ZarjLeanCommand
from zarjcomm.message import ZarjTurnCommand
from zarjcomm.message import ZarjNeckCommand
from zarjcomm.message import ZarjArmCommand
from zarjcomm.message import ZarjHandCommand
from zarjcomm.message import ZarjMovePalmCommand
from zarjcomm.message import ZarjGetPalmCommand
from zarjcomm.message import ZarjGetPalmResponse
from zarjcomm.message import ZarjLocatePointCommand
from zarjcomm.message import ZarjLocatePointResponse
from zarjcomm.message import ZarjGetArmJointsCommand
from zarjcomm.message import ZarjGetArmJointsResponse
from zarjcomm.message import ZarjRequestImageCommand
from zarjcomm.message import ZarjRequestLidarCommand
from zarjcomm.message import ZarjContinueCommand
from zarjcomm.message import ZarjStartWalkCommand
from zarjcomm.limbtypes import LimbTypes
from zarjcomm.anchor import Anchor
from zarjcomm.utils import is_cv3

from task0 import MACROS as MACROS0
from task1 import MACROS as MACROS1
from task2 import MACROS as MACROS2
from task3 import MACROS as MACROS3

class OperatorComputer(object):
    def __init__(self, field_server):
        self.field_server = field_server

        self.sensor_x = 0
        self.sensor_y = 0

        self.clicks = [ None, None ]
        self.anchor = None

        self.task = 0
        self.checkpoint = 0
        self.harness = None
        self.satellite = None
        self.leak = None
        self.lidar_msg = None
        self.lidar_depth = [ 0 ] * 512

        self.msg_queue = Queue.Queue()

        self.builder = gtk.Builder()
        self.builder.add_from_file(os.path.dirname(sys.argv[0]) + "/oc.glade")

        self.zarj_comm = ZarjComm(2823, field_server)
        self.zarj_comm.register(self)

        self.window = self.builder.get_object("mainwindow")

        #Camera section
        self.camera = self.builder.get_object("camera")
        img_pixbuf = gtk.gdk.pixbuf_new_from_file('images/camera.png')
        self.camera.set_from_pixbuf(img_pixbuf)
        self.hazardimage = self.builder.get_object("hazardimage")
        img_pixbuf = gtk.gdk.pixbuf_new_from_file('images/hazard.png')
        self.hazardimage.set_from_pixbuf(img_pixbuf)
        self.hazard_box = self.builder.get_object("hazardbox")
        self.hazard_box.connect("button_press_event", self.hazard_button_cb)
        self.hazard_box.set_events(gtk.gdk.BUTTON_PRESS_MASK)
        self.hazard_image_pixbuf = None
        self.lidar = self.builder.get_object("lidar")
        img_pixbuf = gtk.gdk.pixbuf_new_from_file('images/lidar.png')
        self.lidar.set_from_pixbuf(img_pixbuf)
        self.lidar_events = self.builder.get_object("lidarevents")
        self.lidar_events.connect("motion-notify-event", self.lidar_mouse_cb)
        self.sensor = self.builder.get_object("sensor")
        img_pixbuf = gtk.gdk.pixbuf_new_from_file('images/stereo.png')
        self.sensor.set_from_pixbuf(img_pixbuf)
        self.sensor_box = self.builder.get_object("sensorbox")
        self.have_pixbuf = False
        self.sensor_box.connect("button_press_event", self.sensor_button_cb)
        self.sensor_box.set_events(gtk.gdk.BUTTON_PRESS_MASK)

        # Macro Section
        self.macro_button = self.builder.get_object("executemacro")
        self.macro_button.connect('clicked', self.execute_macro_button_cb)
        self.create_macro_combo()

        # Task control line
        self.create_task_combo()
        self.create_checkpoint_combo()
        self.update_checkpoint_combo()

        self.tell_srcsim_checkbox = self.builder.get_object("tellsrcsim")
        self.deadwalk_checkbox = self.builder.get_object("deadwalkcheck")
        self.start_button = self.builder.get_object("startbutton")
        self.start_button.connect('clicked', self.start_task_button_cb)

        # Stop control line
        self.stop_button = self.builder.get_object("stopmoves")
        self.stop_button.connect('clicked', self.stopmoves_button_cb)
        self.exit_button = self.builder.get_object("exitfc")
        self.exit_button.connect('clicked', self.exitfc_button_cb)
        self.chainokay_checkbox = self.builder.get_object("chainokay")

        # Camera control line
        self.create_camera_controls()

        # Arm control line
        self.limbtypes = LimbTypes()
        self.create_arm_controls()

        # Hand control line
        self.create_hand_controls()

        # Hand movement control line
        self.create_movepalm_controls()

        # Neck and pelvis
        self.neck_lean_amount = self.builder.get_object("necklean")
        self.neck_lean_amount.set_text('0.0')
        self.neck_lean_amount.connect('changed', self.float_entry_changed)
        self.neck_rotate_amount = self.builder.get_object("neckrotate")
        self.neck_rotate_amount.set_text('0.0')
        self.neck_rotate_amount.connect('changed', self.float_entry_changed)
        self.neck_button = self.builder.get_object("neckbutton")
        self.neck_button.connect('clicked', self.neck_button_cb)

        self.pelvis_turn_angle = self.builder.get_object("pelvisturnangle")
        self.pelvis_turn_angle.set_text('0.0')
        self.pelvis_turn_angle.connect('changed', self.float_entry_changed)
        self.turn_button = self.builder.get_object("pelvisturnbutton")
        self.turn_button.connect('clicked', self.turn_button_cb)
        self.pelvis_lean_angle = self.builder.get_object("pelvisleanangle")
        self.pelvis_lean_angle.set_text('0.0')
        self.pelvis_lean_angle.connect('changed', self.float_entry_changed)
        self.lean_button = self.builder.get_object("pelvisleanbutton")
        self.lean_button.connect('clicked', self.lean_button_cb)

        # Walking control
        self.distance = self.builder.get_object("distance")
        self.distance.set_text('0.0')
        self.distance.connect('changed', self.float_entry_changed)
        self.turn = self.builder.get_object("turn")
        self.turn.set_text('0.0')
        self.turn.connect('changed', self.float_entry_changed)
        self.offset = self.builder.get_object("offset")
        self.offset.set_text('0.0')
        self.offset.connect('changed', self.float_entry_changed)
        self.snapto = self.builder.get_object("snapto")
        self.snapto.set_text('15.0')
        self.snapto.connect('changed', self.float_entry_changed)
        self.walkpoi_button = self.builder.get_object("walkbutton")
        self.walkpoi_button.connect('clicked', self.walkpoi_button_cb)
        self.startwalk_button = self.builder.get_object("startwalkbutton")
        self.startwalk_button.connect('clicked', self.startwalk_button_cb)

        self.continuepoi_button = self.builder.get_object("continuebutton")
        self.continuepoi_button.connect('clicked', self.continuepoi_button_cb)

        # Status bar
        self.message_textview = self.builder.get_object("messages")
        self.taskbar = self.builder.get_object("task")
        self.checkpointbar = self.builder.get_object("checkpoint")
        self.harnessbar = self.builder.get_object("harness")
        self.clock = self.builder.get_object("clock")
        self.commstatus = self.builder.get_object("commstatus")
        self.commstats = [ (0, 0) ]
        self.score = self.builder.get_object("score")


        if (self.window):
            self.window.connect("destroy", gtk.main_quit)
            gtk.timeout_add(1000, self.report_comms)

    def main(self, task=None,checkpoint=None):
        self.zarj_comm.start()
        self.window.show_all()
        self.idle_id = gobject.idle_add(self.gtk_idle_cb)

        if task is not None and checkpoint is not None:
            msg = ZarjStartCommand(task, checkpoint, True)
            self.zarj_comm.push_message(msg)

        gtk.main()
        self.zarj_comm.stop()

    @staticmethod
    def format_bps(bytecount, divisor):
        f = float(bytecount) / divisor
        suffix = "Bps"
        if f > 1024:
            suffix = "KBps"
            f /= 1024
        if f > 1024:
            suffix = "MBps"
            f /= 1024

        return "{:5.2f} {}".format(f, suffix)


    def report_comms(self):
        if self.zarj_comm is not None:
            stats = self.zarj_comm.stats()
            self.commstats.append(stats)

            bytes_read = stats[0] - self.commstats[0][0]
            bytes_written = stats[1] - self.commstats[0][1]

            if len(self.commstats) > 5:
                self.commstats.pop(0)

            if self.zarj_comm.connected:
                markup = '<span foreground="green">{}/{}</span>'
            else:
                markup = '<span foreground="red">{}/{}</span>'

            self.commstatus.set_markup(markup.format(
                self.format_bps(bytes_read, len(self.commstats)),
                self.format_bps(bytes_written, len(self.commstats))))
        return True

    # In the zarmcomm thread context...
    def message_ready(self):
        msg = self.zarj_comm.pop_message()
        self.msg_queue.put(msg)

    @staticmethod
    def draw_crosshair(pixmap, x, y, colorname):
        cm = pixmap.get_colormap()
        color = cm.alloc_color(colorname)
        gc = pixmap.new_gc(foreground=color)
        sz = pixmap.get_size()

        left = x - 5
        if left < 0:
            left = 0

        right = x + 5
        if right >= sz[0]:
            right = sz[0] - 1

        top = y - 5
        if top < 0:
            top = 0

        bottom = y + 5
        if bottom >= sz[1]:
            bottom = sz[1] - 1

        pixmap.draw_line(gc, left, y, right, y)
        pixmap.draw_line(gc, x, top, x, bottom)

    def draw_clicks(self):
        pixbuf = self.stereo_image_pixbuf.copy()
        pixmap, mask = pixbuf.render_pixmap_and_mask()
        if self.clicks[0] is not None:
            self.draw_crosshair(pixmap, self.clicks[0][0][0], self.clicks[0][0][1], 'green')
        if self.clicks[1] is not None:
            self.draw_crosshair(pixmap, self.clicks[1][0][0], self.clicks[1][0][1], 'yellow')
        self.sensor.set_from_pixmap(pixmap, mask)

    def lidar_distance_for_angle(self, rad):
        center = len(self.lidar_msg.ranges) / 2
        offset = rad / self.lidar_msg.angle_increment
        if center + offset < 0 or center + offset >= len(self.lidar_msg.ranges):
            return 99.0
        if math.isnan(self.lidar_msg.ranges[int(center + offset)]):
            return 99.0
        return self.lidar_msg.ranges[int(center + offset)]

    def draw_lidar(self):
        if self.lidar_msg is not None:
            pixbuf = gtk.gdk.Pixbuf(gtk.gdk.COLORSPACE_RGB, False, 8, 512, 20)
            a = pixbuf.get_pixels_array()
            camera_increment = (2 * 1.3962) / 512
            offset = (len(self.lidar_msg.ranges) - (2 * 512)) / 2
            scale = 255.0 / (self.lidar_msg.range_max - self.lidar_msg.range_min)
            for x in range(512):
                angle = -1.3962 + (camera_increment * x)
                distance = self.lidar_distance_for_angle(angle)
                self.lidar_depth[x] = round(distance, 2)
                if distance <= self.lidar_msg.range_min:
                    scaled = 0
                elif distance >= self.lidar_msg.range_max:
                    scaled = 255
                else:
                    scaled = distance * scale
                color = [ 0, 255 - int(scaled), 0 ]
                for y in range(20):
                    a[y, x] = color
                if round(math.degrees(angle), 0) % 30 == 0:
                    a[0, x] = [255, 0, 0]
                    a[19, x] = [255, 0, 0]
                if angle == 0.0:
                    a[0, x] = [255, 0, 0]
                    a[1, x] = [255, 0, 0]
                    a[18, x] = [255, 0, 0]
                    a[19, x] = [255, 0, 0]

            pixmap, mask = pixbuf.render_pixmap_and_mask()
            self.lidar.set_from_pixmap(pixmap, mask)

    
    # Ideally, we process in the main thread context, hopefully
    #  preventing bugs...
    def gtk_idle_cb(self):
        if self.msg_queue.empty():
            return True
        
        msg = self.msg_queue.get()
        if isinstance(msg, ZarjStatus):
            self.taskbar.set_text("Task: " + str(msg.task))
            self.task = msg.task
            self.checkpoint = msg.current_checkpoint
            self.harness = msg.harness
            self.update_checkpointbar()
            self.update_harnessbar()
            self.clock.set_text("Task Elapsed Time: {:07.3f}".format(msg.elapsed_time))
            self.score.set_text("Score: " + str(msg.score))

        if isinstance(msg, ZarjMessage):
            buf = self.message_textview.get_buffer()
            buf.insert(buf.get_end_iter(), msg.message)
            self.message_textview.scroll_to_iter(buf.get_end_iter(), 0.0)

        if isinstance(msg, ZarjSatellite):
            self.satellite = msg
            self.update_checkpointbar()

        if isinstance(msg, ZarjLeak):
            self.leak = msg
            self.update_checkpointbar()

        if isinstance(msg, ZarjLaserScan):
            self.lidar_msg = msg
            self.draw_lidar()

        if isinstance(msg, ZarjPicture):
            hazard = msg.description == 'hazard'
            if is_cv3():
                img = cv2.imdecode(msg.picture, cv2.IMREAD_COLOR)
                if not hazard:
                    rgb = cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), -1)
                else:
                    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            else:
                img = cv2.imdecode(msg.picture, cv2.CV_LOAD_IMAGE_COLOR)
                if not hazard:
                    rgb = cv2.flip(cv2.cvtColor(img, cv2.cv.CV_BGR2RGB), -1)
                else:
                    rgb = cv2.cvtColor(img, cv2.cv.CV_BGR2RGB)

            # lidar scan line approxmiate
            if self.lidar_msg is not None and not hazard and \
               not msg.stereo_image:
                cv2.line(rgb, (0, 160), (rgb.shape[1], 160), (255, 0, 0))
                cv2.line(rgb, (rgb.shape[1]/2, 0),
                              (rgb.shape[1]/2, rgb.shape[0]), (0, 155, 155))

            if msg.time is not None:
                txt = "({0:.2f})".format(msg.time)
                cv2.putText(rgb, txt, (0, rgb.shape[0]-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0))

            img_pixbuf = gtk.gdk.pixbuf_new_from_array(rgb, gtk.gdk.COLORSPACE_RGB, 8)
            if msg.stereo_image:
                self.stereo_image_pixbuf = img_pixbuf
                self.clicks = [ None, None ]
                self.draw_clicks()
                self.have_pixbuf = True
            elif hazard:
                self.hazard_image_pixbuf = img_pixbuf
                self.hazardimage.set_from_pixbuf(img_pixbuf)
            else:
                self.camera.set_from_pixbuf(img_pixbuf)

        if isinstance(msg, ZarjLocatePointResponse):
            self.clicks[0][1] = [ msg.x, msg.y, msg.z ]
            self.update_clicked_boxes()

        if isinstance(msg, ZarjGetArmJointsResponse):
            self.on_receive_hand_joints(msg)

        if isinstance(msg, ZarjGetPalmResponse):
            self.on_receive_palm_cb(msg)

        return True

    def update_harnessbar(self):
        markup = "Harness: " + str(self.harness)
        self.harnessbar.set_markup(markup)

    def update_checkpointbar(self):
        markup = "Checkpoint: " + str(self.checkpoint)
        if self.task == 1 and (self.checkpoint == 2 or self.checkpoint == 3):
            if self.satellite is None:
                markup += ' <span foreground="red">No satellite data.</span>'
            else:
                pitch_color = "red"
                if self.satellite.pitch_done and self.satellite.pitch_good:
                    pitch_color = "green"
                elif self.satellite.pitch_done or self.satellite.pitch_good:
                    pitch_color = "yellow"
                good = "Good" if self.satellite.pitch_good else "Bad"
                done = "Done" if self.satellite.pitch_good else "Incomplete"
                markup += ' <span foreground="{}">Pitch: {:1.4f} {} {}</span>'.format(
                    pitch_color, self.satellite.pitch_delta, good, done)

                yaw_color = "red"
                if self.satellite.yaw_done and self.satellite.yaw_good:
                    yaw_color = "green"
                elif self.satellite.yaw_done or self.satellite.yaw_good:
                    yaw_color = "yellow"
                good = "Good" if self.satellite.yaw_good else "Bad"
                done = "Done" if self.satellite.yaw_good else "Incomplete"
                markup += ' <span foreground="{}">Yaw: {:1.4f} {} {}</span>'.format(
                    yaw_color, self.satellite.yaw_delta, good, done)

        if self.task == 3 and self.checkpoint >= 5 and self.checkpoint <= 7:
            if self.leak is None:
                markup += ' <span foreground="red">No leak data.</span>'
            else:
                color = "red"
                if self.leak.value > 0.01:
                    color = "green"
                markup += ' <span foreground="{}">Leak: {:1.4f}</span>'.format(
                    color, self.leak.value)

        self.checkpointbar.set_markup(markup)

    def update_clicked_boxes(self):
        if self.clicks[0] is None or self.clicks[0][1] is None:
            return
        self.clickedx.set_text(str(self.clicks[0][1][0]))
        self.clickedy.set_text(str(self.clicks[0][1][1]))
        self.clickedz.set_text(str(self.clicks[0][1][2]))
        self.update_adjusted()

    def destroy(self, widget, data=None):
        self.zarj_comm.stop()
        gtk.main_quit()

    def lidar_mouse_cb(self, widget, event):
        if self.lidar_msg is None:
            return
        window = widget.get_window()
        if not window:
            return
        real_width = window.get_geometry()[2]
        x = event.x - (real_width - 512) / 2
        if x >= 0 and x <= 511:
            camera_increment = (2 * 1.3962) / 512
            angle = round(math.degrees(-1.3962 + (camera_increment * x)), 0)
            self.lidar.set_tooltip_text(u'{}\xb0\n{}m'.format(angle, self.lidar_depth[ int(x) ]))


    def sensor_button_cb(self, widget, event):
        if self.have_pixbuf:
            self.sensor_x = self.stereo_image_pixbuf.get_width() - int(event.x)
            self.sensor_y = self.stereo_image_pixbuf.get_height() - int(event.y)

            msg = ZarjLocatePointCommand(self.sensor_x, self.sensor_y)
            self.clicks[1] = deepcopy(self.clicks[0])
            self.clicks[0] = [ [ 0, 0], None ]
            self.clicks[0][0][0] = int(event.x)
            self.clicks[0][0][1] = int(event.y)
            self.draw_clicks()
            self.zarj_comm.push_message(msg)

    def hazard_button_cb(self, widget, event):
        size = self.hazardimage.size_request()
        if size[1] > 100:
            self.hazardimage.set_usize(1000, 20)
            size = self.hazardimage.size_request()
            pixbuf = self.hazard_image_pixbuf.scale_simple(
                size[0], size[1], gtk.gdk.INTERP_BILINEAR)
            self.hazardimage.set_from_pixbuf(pixbuf)
        elif self.hazard_image_pixbuf is not None:
            self.hazardimage.set_usize(1000, 250)
            size = self.hazardimage.size_request()
            pixbuf = self.hazard_image_pixbuf.scale_simple(
                size[0], size[1], gtk.gdk.INTERP_BILINEAR)
            self.hazardimage.set_from_pixbuf(pixbuf)

    def create_task_combo(self):
        self.task_combo = gtk.combo_box_new_text()
        self.taskmenubox = self.builder.get_object("taskmenubox")
        self.taskmenubox.add(self.task_combo)
        self.task_combo.append_text("1 - Communication Dish")
        self.task_combo.append_text("2 - Solar Array")
        self.task_combo.append_text("3 - Air Leak")
        self.task_combo.set_active(0)
        self.task_combo.connect('changed', self.task_changed_cb)

    def create_macro_combo(self):
        self.macro_combo = gtk.combo_box_new_text()
        self.macromenubox = self.builder.get_object("macrocombobox")
        self.macromenubox.add(self.macro_combo)

        self.macro_combo.append_text('--- Select a macro ---')
        self.macro_combo.set_active(0)

    def task_changed_cb(self, combobox):
        self.update_checkpoint_combo()

    def create_checkpoint_combo(self):
        self.checkpoint_combo = gtk.combo_box_new_text()
        self.checkpointmenubox = self.builder.get_object("checkpointmenubox")
        self.checkpointmenubox.add(self.checkpoint_combo)

    def update_checkpoint_combo(self):
        self.checkpoint_combo.get_model().clear()
        if self.task_combo.get_active() == 0:
            self.checkpoint_combo.append_text("1 - Walk to Dish")
            self.checkpoint_combo.append_text("2 - Set Pitch or Yaw")
            self.checkpoint_combo.append_text("3 - Set Pitch and Yaw")
            self.checkpoint_combo.append_text("4 - Finish Box")
        if self.task_combo.get_active() == 1:
            self.checkpoint_combo.append_text("1 - Get Solar Panel")
            self.checkpoint_combo.append_text("2 - Place Solar Panel")
            self.checkpoint_combo.append_text("3 - Deploy Panel")
            self.checkpoint_combo.append_text("4 - Pick up Cable")
            self.checkpoint_combo.append_text("5 - Plug in Cable")
            self.checkpoint_combo.append_text("6 - Finish box")
        if self.task_combo.get_active() == 2:
            self.checkpoint_combo.append_text("1 - Climb Stairs")
            self.checkpoint_combo.append_text("2 - Open Door")
            self.checkpoint_combo.append_text("3 - Pass through Door")
            self.checkpoint_combo.append_text("4 - Pick up Leak Detector")
            self.checkpoint_combo.append_text("5 - Find the leak")
            self.checkpoint_combo.append_text("6 - Pick up leak repair tool")
            self.checkpoint_combo.append_text("7 - Repair the leak")
            self.checkpoint_combo.append_text("8 - Finish box")
        self.checkpoint_combo.set_active(0)

        self.macro_combo.get_model().clear()
        self.macro_combo.append_text('--- Select a macro ---')
        for macro in MACROS0:
            self.macro_combo.append_text(macro)
        if self.task_combo.get_active() == 0:
            for macro in MACROS1:
                self.macro_combo.append_text(macro)
        elif self.task_combo.get_active() == 1:
            for macro in MACROS2:
                self.macro_combo.append_text(macro)
        elif self.task_combo.get_active() == 2:
            for macro in MACROS3:
                self.macro_combo.append_text(macro)
        self.macro_combo.set_active(0)

    def start_task_button_cb(self, button):
        msg = ZarjStartCommand(self.task_combo.get_active() + 1,
                self.checkpoint_combo.get_active() + 1,
                self.tell_srcsim_checkbox.get_active())
        self.zarj_comm.push_message(msg)

    def stopmoves_button_cb(self, button):
        msg = ZarjStopCommand()
        self.zarj_comm.push_message(msg)

    def exitfc_button_cb(self, button):
        label = gtk.Label("Are you sure you want the FC to exit?")
        dialog = gtk.Dialog("Confirm Exit",
                       None,
                       gtk.DIALOG_MODAL | gtk.DIALOG_DESTROY_WITH_PARENT,
                       (gtk.STOCK_CANCEL, gtk.RESPONSE_REJECT,
                       gtk.STOCK_OK, gtk.RESPONSE_ACCEPT))
        dialog.vbox.pack_start(label)
        label.show()
        response = dialog.run()
        dialog.destroy()

        if response == gtk.RESPONSE_ACCEPT:
            msg = ZarjExitCommand()
            self.zarj_comm.push_message(msg)

    def execute_macro_button_cb(self, button):
        msg = ZarjMacroCommand(self.macro_combo.get_active_text(),
                self.chainokay_checkbox.get_active())
        self.zarj_comm.push_message(msg)

    def request_image_cb(self, button):
        msg = ZarjRequestImageCommand(False, True)
        self.zarj_comm.push_message(msg)

    def request_regular_image_cb(self, button):
        msg = ZarjRequestImageCommand(True, False)
        self.zarj_comm.push_message(msg)

    def pictureinterval_cb(self, button):
        msg = ZarjIntervalCommand(float(self.picture_interval.get_text()))
        self.zarj_comm.push_message(msg)

    def request_hazard_image_cb(self, button):
        msg = ZarjRequestImageCommand(False, False, True)
        self.zarj_comm.push_message(msg)
        self.hazardimage.set_usize(1000, 250)
        img_pixbuf = gtk.gdk.pixbuf_new_from_file('images/wait.png')
        self.hazard_image_pixbuf = img_pixbuf
        self.hazardimage.set_from_pixbuf(img_pixbuf)

    def request_lidar_cb(self, button):
        msg = ZarjRequestLidarCommand()
        self.zarj_comm.push_message(msg)

    def create_arm_controls(self):
        self.arm_combo_box = self.builder.get_object("armcombobox")

        self.arm_side_combo = gtk.combo_box_new_text()
        self.arm_combo_box.add(self.arm_side_combo)
        self.arm_side_combo.append_text("left")
        self.arm_side_combo.append_text("right")
        self.arm_side_combo.set_active(0)

        self.arm_combo = gtk.combo_box_new_text()
        self.arm_combo_box.add(self.arm_combo)
        self.arm_combo.append_text('--- Select an arm configuration ---')
        for style in sorted(LimbTypes.arm_styles.iterkeys()):
            self.arm_combo.append_text(style)
        self.arm_combo.set_active(0)
        self.arm_combo.connect('changed', self.arm_style_changed_cb)

        self.arm_text_boxes = []
        for arm_entry in self.limbtypes.arm_descriptions:
            arm_text_box = self.builder.get_object(arm_entry[0])
            arm_text_box.set_text('0.0')
            arm_text_box.connect('changed', self.float_entry_changed)
            self.arm_text_boxes.append(arm_text_box)

        self.arm_configuration_button = self.builder.get_object("armconfigurationbutton")
        self.arm_configuration_button.connect('clicked', self.arm_configuration_cb)
        self.arm_configuration_load_button = self.builder.get_object("armconfigurationloadbutton")
        self.arm_configuration_load_button.connect('clicked', self.arm_configuration_load_cb)

    def create_camera_controls(self):
        self.clickedx = self.builder.get_object("clickedx")
        self.clickedx.set_text('0.0')
        self.clickedy = self.builder.get_object("clickedy")
        self.clickedy.set_text('0.0')
        self.clickedz = self.builder.get_object("clickedz")
        self.clickedz.set_text('0.0')

        self.anchor_combo = gtk.combo_box_new_text()
        self.anchormenubox = self.builder.get_object("anchorcombobox")
        self.anchormenubox.add(self.anchor_combo)
        self.anchor_combo.connect('changed', self.anchor_changed_cb)

        self.anchor_combo.append_text('--- Select an anchor ---')
        for anchor in sorted(Anchor.list()):
            self.anchor_combo.append_text(anchor)
        self.anchor_combo.set_active(0)

        self.adjustedx = self.builder.get_object("adjustedx")
        self.adjustedx.set_text('0.0')
        self.adjustedy = self.builder.get_object("adjustedy")
        self.adjustedy.set_text('0.0')
        self.adjustedz = self.builder.get_object("adjustedz")
        self.adjustedz.set_text('0.0')
        self.adjustedangle = self.builder.get_object("adjustedangle")
        self.adjustedangle.set_text('0.0')

        self.request_image_button = self.builder.get_object("requestimagebutton")
        self.request_image_button.connect('clicked', self.request_image_cb)

        self.picture_interval = self.builder.get_object("pictureinterval")
        self.picture_interval.set_text('0.0')
        self.picture_interval.connect('changed', self.float_entry_changed)
        self.picture_interval_button = self.builder.get_object("pictureintervalbutton")
        self.picture_interval_button.connect('clicked', self.pictureinterval_cb)
        self.request_regular_image_button = self.builder.get_object("requestregularbutton")
        self.request_regular_image_button.connect('clicked', self.request_regular_image_cb)
        self.hazard_regular_image_button = self.builder.get_object("requesthazardbutton")
        self.hazard_regular_image_button.connect('clicked', self.request_hazard_image_cb)

        self.request_lidar_button = self.builder.get_object("requestlidarbutton")
        self.request_lidar_button.connect('clicked', self.request_lidar_cb)

    def anchor_changed_cb(self, combobox):
        self.update_adjusted()

    def update_adjusted(self):
        if self.anchor is None or self.anchor.name != self.anchor_combo.get_active_text():
            self.anchor = Anchor.get(self.anchor_combo.get_active_text())

        if self.anchor is None or self.anchor.adjusted is None:
            return

        point1 = None
        point2 = None
        if self.clicks[0] is not None and self.clicks[0][1] is not None:
            point1 = self.clicks[0][1]
        if self.clicks[1] is not None and self.clicks[1][1] is not None:
            point2 = self.clicks[1][1]

        self.anchor.update(point1, point2)
        self.adjustedx.set_text("{0:.3f}".format(self.anchor.adjusted[0]))
        self.adjustedy.set_text("{0:.3f}".format(self.anchor.adjusted[1]))
        self.adjustedz.set_text("{0:.3f}".format(self.anchor.adjusted[2]))
        if self.anchor.angle is not None:
            self.adjustedangle.set_text("{0:.3f}".format(self.anchor.angle))

    def arm_style_changed_cb(self, combobox):
        sidename = self.arm_side_combo.get_active_text()
        stylename = self.arm_combo.get_active_text()
        if stylename is None:
            return

        joints = deepcopy(LimbTypes.arm_styles[stylename])
        if not joints is None:
            i = 0
            for j in joints:
                self.arm_text_boxes[i].set_text(str(j))
                i = i + 1

    def arm_configuration_cb(self, button):
        joints = []
        i = 0
        for j in self.arm_text_boxes:
            val = float(j.get_text())
            if not math.isnan(val):
                if val < self.limbtypes.arm_descriptions[i][1][0]:
                    val = self.limbtypes.arm_descriptions[i][1][0]
                    j.set_text(str(val))
                if val > self.limbtypes.arm_descriptions[i][1][1]:
                    val = self.limbtypes.arm_descriptions[i][1][1]
                    j.set_text(str(val))
            joints.append(val)
            i = i + 1
        sidename = self.arm_side_combo.get_active_text()
        msg = ZarjArmCommand(sidename,
                self.limbtypes.invert_arm_configuration(sidename, joints))
        self.zarj_comm.push_message(msg)

    def arm_configuration_load_cb(self, button):
        sidename = self.arm_side_combo.get_active_text()
        msg = ZarjGetArmJointsCommand(sidename)
        self.zarj_comm.push_message(msg)

    def create_hand_controls(self):
        self.hand_combo_box = self.builder.get_object("handcombobox")

        self.hand_side_combo = gtk.combo_box_new_text()
        self.hand_combo_box.add(self.hand_side_combo)
        self.hand_side_combo.append_text("left")
        self.hand_side_combo.append_text("right")
        self.hand_side_combo.set_active(0)

        self.hand_combo = gtk.combo_box_new_text()
        self.hand_combo_box.add(self.hand_combo)
        self.hand_combo.append_text('--- Select a hand configuration ---')
        for style in sorted(LimbTypes.hand_styles.iterkeys()):
            self.hand_combo.append_text(style)
        self.hand_combo.set_active(0)
        self.hand_combo.connect('changed', self.hand_style_changed_cb)

        self.hand_text_boxes = []
        for hand_entry in self.limbtypes.hand_descriptions:
            hand_text_box = self.builder.get_object(hand_entry[0])
            hand_text_box.set_text('0.0')
            hand_text_box.connect('changed', self.float_entry_changed)
            self.hand_text_boxes.append(hand_text_box)

        self.hand_configuration_button = self.builder.get_object("handconfigurationbutton")
        self.hand_configuration_button.connect('clicked', self.hand_configuration_cb)

    def hand_style_changed_cb(self, combobox):
        sidename = self.hand_side_combo.get_active_text()
        stylename = self.hand_combo.get_active_text()
        joints = deepcopy(LimbTypes.hand_styles[stylename])
        if not joints == None:
            i = 0
            for j in joints:
                self.hand_text_boxes[i].set_text(str(j))
                i = i + 1

    def on_receive_hand_joints(self, joints_response):
        if isinstance(joints_response, ZarjGetArmJointsResponse):
            values = self.limbtypes.invert_arm_configuration(joints_response.sidename, joints_response.values)
            sidename = self.arm_side_combo.get_active_text()
            if sidename == joints_response.sidename and not values is None:
                for i in range(0,len(values)):
                    self.arm_text_boxes[i].set_text("{0:.2f}".format(values[i]))

    def hand_configuration_cb(self, button):
        joints = []
        i = 0
        for j in self.hand_text_boxes:
            val = float(j.get_text())
            if not math.isnan(val):
                if val < self.limbtypes.hand_descriptions[i][1][0]:
                    val = self.limbtypes.hand_descriptions[i][1][0]
                    j.set_text(str(val))
                if val > self.limbtypes.hand_descriptions[i][1][1]:
                    val = self.limbtypes.hand_descriptions[i][1][1]
                    j.set_text(str(val))
            joints.append(val)
            i = i + 1

        sidename = self.hand_side_combo.get_active_text()
        msg = ZarjHandCommand(sidename,
                self.limbtypes.invert_hand_configuration(sidename, joints))
        self.zarj_comm.push_message(msg)


    def create_movepalm_controls(self):
        self.movepalm_combo_box = self.builder.get_object("movepalmcombobox")
        self.movepalm_side_combo = gtk.combo_box_new_text()
        self.movepalm_combo_box.add(self.movepalm_side_combo)
        self.movepalm_side_combo.append_text("left")
        self.movepalm_side_combo.append_text("right")
        self.movepalm_side_combo.set_active(0)

        self.relative_checkbox = self.builder.get_object("handrelativecheck")

        self.handx = self.builder.get_object("handx")
        self.handx.set_text('0.0')
        self.handx.connect('changed', self.float_entry_changed)
        self.handy = self.builder.get_object("handy")
        self.handy.set_text('0.0')
        self.handy.connect('changed', self.float_entry_changed)
        self.handz = self.builder.get_object("handz")
        self.handz.set_text('0.0')
        self.handz.connect('changed', self.float_entry_changed)

        self.palmyaw = self.builder.get_object("palmyaw")
        self.palmyaw.set_text('0.0')
        self.palmyaw.connect('changed', self.float_entry_changed)
        self.palmpitch = self.builder.get_object("palmpitch")
        self.palmpitch.set_text('0.0')
        self.palmpitch.connect('changed', self.float_entry_changed)
        self.palmroll = self.builder.get_object("palmroll")
        self.palmroll.set_text('0.0')
        self.palmroll.connect('changed', self.float_entry_changed)

        self.move_palm_button = self.builder.get_object("movepalmcenter")
        self.move_palm_button.connect('clicked', self.move_palm_cb)
        self.get_palm_button = self.builder.get_object("getpalmcenter")
        self.get_palm_button.connect('clicked', self.get_palm_cb)

    def move_palm_cb(self, button):
        msg = ZarjMovePalmCommand(self.movepalm_side_combo.get_active_text(),
                self.relative_checkbox.get_active(),
                float(self.handx.get_text()),
                float(self.handy.get_text()),
                float(self.handz.get_text()),
                float(self.palmyaw.get_text()),
                float(self.palmpitch.get_text()),
                float(self.palmroll.get_text()),
                True)
        self.zarj_comm.push_message(msg)

    def get_palm_cb(self, button):
        msg = ZarjGetPalmCommand(self.movepalm_side_combo.get_active_text())
        self.zarj_comm.push_message(msg)

    def on_receive_palm_cb(self, palm_response):
        if isinstance(palm_response, ZarjGetPalmResponse):
            sidename = self.movepalm_side_combo.get_active_text()
            if sidename == palm_response.sidename and not self.relative_checkbox.get_active():
                self.handx.set_text("{0:.2f}".format(palm_response.x))
                self.handy.set_text("{0:.2f}".format(palm_response.y))
                self.handz.set_text("{0:.2f}".format(palm_response.z))
                self.palmyaw.set_text("{0:.2f}".format(palm_response.yaw))
                self.palmpitch.set_text("{0:.2f}".format(palm_response.pitch))
                self.palmroll.set_text("{0:.2f}".format(palm_response.roll))

    def neck_button_cb(self, button):
        msg = ZarjNeckCommand([ 
                float(self.neck_lean_amount.get_text()),
                float(self.neck_rotate_amount.get_text()),
                0.0 ])
        self.zarj_comm.push_message(msg)

    def lean_button_cb(self, button):
        msg = ZarjLeanCommand(float(self.pelvis_lean_angle.get_text()))
        self.zarj_comm.push_message(msg)

    def turn_button_cb(self, button):
        msg = ZarjTurnCommand(float(self.pelvis_turn_angle.get_text()))
        self.zarj_comm.push_message(msg)

    def walkpoi_button_cb(self, button):
        msg = ZarjWalkCommand(float(self.distance.get_text()),
                float(self.turn.get_text()),
                float(self.offset.get_text()),
                self.deadwalk_checkbox.get_active(),
                float(self.snapto.get_text()))
        self.zarj_comm.push_message(msg)
        self.distance.set_text('0.0')
        self.turn.set_text('0.0')
        self.offset.set_text('0.0')

    def startwalk_button_cb(self, button):
        msg = ZarjStartWalkCommand()
        self.zarj_comm.push_message(msg)

    def continuepoi_button_cb(self, button):
        msg = ZarjContinueCommand()
        self.zarj_comm.push_message(msg)

    def float_entry_changed(self, entry):
        try:
            val = float(entry.get_text())
            entry.modify_text(gtk.STATE_NORMAL, gtk.gdk.color_parse("black"))
        except ValueError:
            entry.modify_text(gtk.STATE_NORMAL, gtk.gdk.color_parse("red"))


if __name__ == "__main__":
    field_server = "localhost"
    task = None
    checkpoint = None
    for arg in sys.argv[1:]:
        if arg[:5] == "task=":
            task = int(arg[5:])
        elif arg[:11] == "checkpoint=":
            checkpoint = int(arg[11:])
        else:
            field_server = arg
    operator = OperatorComputer(field_server)
    operator.main(task, checkpoint)
