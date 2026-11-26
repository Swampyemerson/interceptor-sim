# Target tag for a letter-size sheet

One file, the current design only.

| File | What | Size, measured from the file |
|---|---|---|
| `tag36h11_id0_letter.pdf` | AprilTag family tag36h11, id 0, the tag the flight code looks for by default | US letter page (8.5 x 11 in). Black square 159.85 mm edge to edge. White margin 28 mm each side, 23 mm above. A 100 mm check bar is printed below it. |

- Print at 100 % ("actual size"), not "fit to page".
- Measure the black square edge to edge with a ruler after printing. That measured
  number, in metres, is what the flight program is given as `--target-span-m`. A
  printer that scales by 3 % moves every range the camera reports by 3 %.
- Do not trim the white border closer than 20 mm to the black, and mount it flat on
  something stiff: a curled tag reads as a different size and angle.
- Checked 2026-11-26: the page image, shrunk to one sixth and also turned upside down,
  decodes as id 0 with no bit errors in the library the flight code uses on the laptop
  (`pupil_apriltags`). It has not been decoded through the nose camera.

Generator: the command is in the commit that added this folder
(`cv2.aruco` dictionary `DICT_APRILTAG_36h11`, 236 px per cell at 300 dpi).
