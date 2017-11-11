
thickness = 2.4;
size = [46+2*thickness, 22+2*thickness, 35];    // outer dimensions
connSize = [13, 13, 12.0];
usbSize = [8.5, 8.5, 5];
pcbThickness = 1.6;
connSize2 = [11, 11, 2.5];

$fn = 32;


module box(size, thickness, radius=4, tolerance=0.2, lid=false) {
    // some shorter aliases
    t = tolerance;
    th = thickness;
    r = radius;
    d = 2*r;

    // lid slope
    slope1 = 1.5;
    slope2 = 0.5;

    module outerCorner(x, y) {
        translate([(size[0] - d + t)*x, (size[1] - d + t)*y, 0])
            cylinder(size[2] + th, r=r);
    }

    module innerCorner(x, y) {
        translate([(size[0] - d + t)*x, (size[1] - d + t)*y, 0])
            cylinder(size[2], r=r-th);
    }

    module lidGrooveCorner(x, y) {
        translate([(size[0] - d + th - t)*x, (size[1] - d + t)*y, 0])
            cylinder(th, r - th + slope1, r - th + slope2);
    }

    module lidCorner(x, y) {
        translate([(size[0] - d + t)*x, (size[1] - d + t)*y, 0])
            cylinder(th, r - th + slope1 - t, r - th + slope2 - t);
    }

    translate([r, r, 0])
        if (lid) {
            // draw lid only
            hull() { for (i=[0, 1], j=[0, 1]) lidCorner(i, j); }
        } else {
            // draw box only
            difference () {
                hull() { for (i=[0, 1], j=[0, 1]) outerCorner(i, j); }

                translate([0, 0, th])
                    hull() { for (i=[0, 1], j=[0, 1]) innerCorner(i, j); }

                translate([0, 0, size[2] + t])
                    hull() { for (i=[0, 1], j=[0, 1]) lidGrooveCorner(i, j); }

                translate([size[0] - d, -r, size[2] + t])
                    cube([r, size[1] + t, th]);
            }
        }
}

module lid(size, thickness=2, radius=4, tolerance=0.2) {
    box(size, thickness, radius, tolerance, lid=true);
}

module pcbBox() {
    difference() {
        th = thickness;

        union() {
            translate([0, -size[1]/2, 0]) box(size, th);
        }
        // connectors
        usbPos = connSize[2] + pcbThickness + 4 + 8;
        translate([-.1, -usbSize[1]/2, th + usbPos]) cube(usbSize);
        translate([-.1, -connSize[1]/2, th]) cube(connSize);
        translate([size[0]-connSize2[0]/2, -connSize2[1]/2, th+3]) {
            cube(connSize2);
            translate([0, 0, 2.54*2]) cube(connSize2);
        }
    }
}

module wedge() {
    union() {
        cube([2.2, 12, 8]);
        cube([5.0, 12, 6]);
    }
}

pcbBox();
translate([0, size[1], 0]) lid(size, thickness);
//wedge();
