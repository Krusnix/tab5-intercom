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

- `tab5_video` (coming next): shows the other person's picture during a
  VoIP Stack video call on a Tab5 whose screen is drawn with a normal
  ESPHome display lambda (no LVGL).
