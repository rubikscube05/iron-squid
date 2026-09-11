/* 
   Filleted Triangular Bipyramid
   (Triangle extruded to a point on BOTH sides)
   1 unit = 1 mm
*/

// --- Parameters ---
side_length = 500;       // Side length of the equilateral triangle base (mm)
extrusion_height = 200;  // Height from the center plane to EACH tip (mm)
fillet_radius = 5;       // Radius of the rounded edges (mm)

// Resolution settings
$fn = 60;                // Global resolution for smooth curves

// --- Calculations ---
// Circumradius of an equilateral triangle is (side / sqrt(3))
target_radius = side_length / sqrt(3);

// Shrink the inner template so the Minkowski sphere adds exactly the right thickness back
// The corners of a triangle expand by 2x the radius when offset.
inner_radius = target_radius - (2 * fillet_radius);

// The tip expands by exactly 1x the radius straight up/down
inner_height = extrusion_height - fillet_radius;

// --- 3D Model ---
// The shape is built centered on the Z=0 plane (XY plane)
minkowski() {
    // Inner core bipyramid
    union() {
        // Top pyramid (Extrudes from Z=0 up to Z=inner_height)
        cylinder(
            h = inner_height, 
            r1 = inner_radius, 
            r2 = 0, 
            $fn = 3
        );
        
        // Bottom pyramid (Mirrored to extrude from Z=0 down to Z=-inner_height)
        mirror([0, 0, 1]) {
            cylinder(
                h = inner_height, 
                r1 = inner_radius, 
                r2 = 0, 
                $fn = 3
            );
        }
    }
    
    // The sphere that acts like a 3D brush to round everything
    sphere(r = fillet_radius);
}
