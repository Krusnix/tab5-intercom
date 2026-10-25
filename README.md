# tab5-intercom

Add-on code for Trent's two M5Stack Tab5 intercoms (ESPHome + VoIP Stack
through Home Assistant).

The Tab5 firmware YAML lives in Home Assistant's ESPHome Builder. It pulls
the components in this repository with:

```yaml
external_components:
  - source: github://Krusnix/tab5-intercom@<tag>
    components: [tab5_video]
```

Always pin a tag, so a later change here never alters a build by surprise.

## Components

### `tab5_video`

Video calls for a VoIP Stack (`n-IA-hane/esphome-voip-stack`, JPEG video
profile) ESP32-P4 device whose screen is drawn with a normal ESPHome display
lambda. VoIP Stack's own `p4_video_renderer` needs LVGL, and ESPHome does not
allow LVGL next to a display lambda, so this component does the receive side
without LVGL:

- Each received JPEG picture is decoded by the P4's hardware JPEG decoder on
  a small task of its own (stack in PSRAM).
- The ESPHome main loop scales it with the PPA to fill a rectangle of the
  screen (cutting off at most `max_crop`, otherwise with black bars), turns it
  to match the display's rotation, and writes it straight to the panel. All
  panel writes stay on the main loop, like the display lambda's.
- `has_frame()` is true while a picture is on the screen, so the lambda can
  draw only what is outside the picture. `on_first_frame` and
  `on_video_ended` tell it when to change layout. The picture goes off when
  the call's video ends or when no picture arrived for `no_video_timeout`.

The optional `camera:` block adds a gate in front of the camera's
`esp_jpeg_video_source`. Give the gate to VoIP Stack as `video.source`.
VoIP Stack can only change video direction during a call if video was agreed
when the call started, and it agrees to send only while sending is wanted.
With the gate, VoIP Stack always agrees on video both ways, while the camera
itself stays off at the start of every call. `set_enabled(true)` starts it
during a call; it stops again by itself when the call's video ends.

The gate also hands every picture it sends to the view's **self view**: the
device's own picture, scaled down and mirrored (like a mirror; the other side
sees it the right way round). The display lambda says where it goes with
`id(remote_video).set_self_view(x, y, w, h)` on every drawing (0 width hides
it); it is drawn again right after each screen update, and over the other
side's picture it is put into that picture before it reaches the panel, so
it never flickers. Turn it off with `self_view: false`.

```yaml
esp_jpeg_video_source:
  id: camera_video
  camera_id: tab5_camera
  width: 640
  height: 360
  framerate: 10

tab5_video:
  id: remote_video
  display_id: main_display
  display_rotation: 270        # same as the display's rotation:
  area: {x: 0, y: 0, width: 1280, height: 620}
  width: 640                   # picture size asked of the other side
  height: 360
  framerate: 10
  # max_decode_width: 1280     # largest picture accepted from any caller
  # max_decode_height: 720
  # max_crop: 15%
  # no_video_timeout: 3s
  # swap_red_blue: false       # if red and blue come out swapped
  camera:
    id: camera_gate
    source_id: camera_video
    # self_view: true          # own picture where set_self_view() says
    # self_view_mirror: true
    # self_view_framerate: 10
    # width: 640               # the camera's call picture size
    # height: 360
  on_first_frame:
    - component.update: main_display
  on_video_ended:
    - component.update: main_display

voip_stack:
  video:
    codec: jpeg
    source: camera_gate
    sink: remote_video
    width: 640
    height: 360
    framerate: 10
    rtp_port: 40002
    offer_payload_type: 26
    max_rtp_payload: 1400
```

In lambdas: `id(remote_video).has_frame()`, `is_receiving()`,
`frames_shown()`, `decode_errors()`, `set_self_view(x, y, w, h)`,
`get_self_view(&x, &y, &w, &h)`, and `id(camera_gate).set_enabled(bool)`,
`is_enabled()`, `is_available()`.

Note for the display lambda (built-in `mipi_dsi`): that driver only sends
pixels that changed in its own buffer. After the picture goes away, mark the
whole screen as changed before drawing (for example by drawing two corner
pixels in two different colours), or parts of the last picture stay. The
same goes for the self view's old rectangle when it moves or is hidden.

Built against ESPHome 2026.10.0, VoIP Stack v2026.11.0-dev and
esphome-esp-video-camera v2026.10.0-dev for the M5Stack Tab5.
