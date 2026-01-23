# GreasePad 1.2.1

The *GreasePad* provides a drawing area and expects pen strokes as the only user input. These strokes are approximated by straight line segments, resulting in line drawings. During this interactive process, geometric relations such as orthogonality are recognized and enforced immediately through an adjustment process. The program can be utilized to study systems of straight lines and constraints (closure theorems) or to outline the shapes of human-made objects in images. As a pointing device, you can use a computer mouse, a stylus, or one of your fingers in combination with a touchscreen.

<figure>
<img src="./orthocenter.gif" width = "100%">
</figure>

## Content
- [News](#news)
- [Precompiled Win64 binaries](#pre-compiled)
- [Gallery (Screenshots)](#gallery)
- [Copyright](#copyright)
- [Licensing](#licensing)
- [Contact](#contact)
- [References](#references)
- [Acknowledgements](#acknowledgements)
- [Frequently Asked Questions](#faq)

<a name="news"></a>
## News

* Version 1.2.1 (November 2025): The source code has been completely revised and modernized. Minor bugs have been fixed.

* Version 1.2.0 (October 2023): The additional, specific constraints *horizontal*, *vertical*, and *diagonal* enable the alignment of segments with the frame of the canvas. These constraints are disabled by default; they must be activated by selecting the corresponding menu item.

<a name="pre-compiled"></a>
## Pre-compiled Win64 binaries

No installation procedure is provided. Instead, please perform the following steps:
                                                              
1. Download the zip file from [https://FraunhoferIOSB.github.io/GreasePad/downloads/GreasePad121_x64.zip](https://FraunhoferIOSB.github.io/GreasePad/downloads/GreasePad120_x64.zip) to your machine. 
2. Unpack the file contents into a folder. The zip file contains the executable *greasepad.exe* and the required dynamically linked libraries.
3. Select and start the executable *greasepad.exe*.


<a name="gallery"></a>
## Gallery (Screenshots)

### Pappus's hexagon theorem

<figure>
<img src="./gallery/pappus_affine.png" width = "100%">
<figcaption align="left">
Pappus's hexagon theorem, affine form
</figcaption>
</figure>

### Confidence regions

<figure>
<img src="./gallery/confidence.png" width = "100%">
<figcaption align="left">
Confidence regions
</figcaption>
</figure>

### Tracing and Outlining

<figure>
<img src="./gallery/outlining.png" width = "100%">
<figcaption align="left">
Tracing and outlining
</figcaption>
</figure>

### Orthocenter of a Triangle

<figure>
<img src="./gallery/orthocenter.png" width = "100%">
<figcaption align="left">
Orthocenter of a triangle
</figcaption>
</figure>


### Identification of Subtasks

<figure>
<img src="./gallery/subtasks.png" width = "100%">
<figcaption align="left">
Identification of subtasks (connected components)
</figcaption>
</figure>

### Thomsen's Figure

<figure>
<img src="./gallery/thomsen.png" width = "100%">
<figcaption align="left">
Thomsen's figure
</figcaption>
</figure>

### Mid Point of a Triangle's Side

<figure>
<img src="./gallery/mid_point.png" width = "100%">
<figcaption align="left">
Mid point of a triangle's side using a trapezoid
</figcaption>
</figure>




<a name="copyright"></a>
## Copyright

Copyright (c) 2022-2026 Jochen Meidow, [Fraunhofer IOSB](http://www.iosb.fraunhofer.de), Germany.


<a name="licensing"></a>
## Licensing

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either [version 3 of the License](https://www.gnu.org/licenses/gpl-3.0.en.html), or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <https://www.gnu.org/licenses/>.



<a name="contact"></a>
## Contact

Jochen Meidow, Fraunhofer IOSB, Ettlingen, Germany.

<jochen.meidow@iosb.fraunhofer.de>


<a name="references"></a>
## References

Details on the utilized methods can be found in the following papers:

- J. Meidow (2023) [Geometric Reasoning for the freehand-drawn Delineation of 2D Geometries](https://doi.org/10.1016/j.isprsjprs.2023.05.001). ISPRS Journal of Photogrammetry and Remote Sensing, vol. 201, pp. 67-77
- J. Meidow and L. Lucks (2019) [Draw and Order - Modeless Interactive Acquisition of Outlines](https://doi.org/10.5194/isprs-annals-IV-2-W7-103-2019). ISPRS - Annals of Photogrammetry, Remote Sensing and Spatial Information Sciences, vol. IV-2/W7, pp. 103-110
- J. Meidow, H. Hammer, and L. Lucks (2020) [Delineation and Construction of 2D Geometries by Freehand Drawing and Geometric Reasoning](https://www.isprs-ann-photogramm-remote-sens-spatial-inf-sci.net/V-5-2020/77/2020/). ISPRS - Annals of the Photogrammetry, Remote Sensing and Spatial Information Sciences, vol. V-5-2020, pp. 77-84
- J. Meidow and H. Hammer (2025) [Minimal rotations in arbitrary dimensions with applications to hypothesis testing and parameter estimation](https://doi.org/10.1016/j.ophoto.2025.100085). ISPRS Open Journal of Photogrammetry and Remote Sensing, vol. 15, p. 100085


<a name ="acknowledgments"></a>
## Acknowledgements

The author would like to thank Wolfgang Förstner (University of Bonn) and Horst Hammer (Fraunhofer IOSB) for their inspiring collaboration.

<a name ="faq"></a>
## Frequently Asked Questions

### Interaction

*Q. Given a scene with segments and constraints, how do I select individual segments and/or constraints?*

A. With a computer mouse as a pointing device, you can select segments or constraints using the mouse's right button. With a stylus or one of your fingers, you will probably have to press for a short moment onto the touchpad. If a graphical element covers another element, you can change the visual stacking.
 
### Further geometric elements

*Q. Why are there no curve segments of higher order provided, say arcs of ellipses or arcs of circles?* 

A. The joint utilization of curves and straight lines implies a model selection. Actually, one cannot distinguish automatically and reliably between a short sequence representing a straight line segment and a short sequence representing a short arc of a curve. For this to work, the users would have to express their intentions interactively. This approach, however, is in conflict with the modeless design of the program.