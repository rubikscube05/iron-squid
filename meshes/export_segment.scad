// export_segment.scad
// Run via CLI to batch export centered individual segments

i = 0; // Default index overridden by CLI

num_sections = 20;
base_side = 100;
base_top_h = 30;
base_bot_h = 30;
base_top_face = 10;
base_bot_face = 10;
base_fillet = 5.0;

spine_radius = 2.5; 
cable_radius = 1.5; 

function get_scale(idx) = 1.0 - (idx * (0.65 / (num_sections - 1)));

module single_segment(idx) {
    s = get_scale(idx);
    
    c_side = base_side * s;
    c_top_h = base_top_h * s;
    c_bot_h = base_bot_h * s;
    c_top_face = base_top_face * s;
    c_bot_face = base_bot_face * s;
    c_fillet = base_fillet * s;
    
    radius = c_side / sqrt(3); 
    core_radius = max(0.1, radius - c_fillet); 
    core_top = max(0.1, c_top_face - c_fillet);
    core_bot = max(0.1, c_bot_face - c_fillet);

    difference() {
        minkowski() {
            union() {
                cylinder(h=c_top_h - c_fillet, r1=core_radius, r2=core_top, $fn=3);
                rotate([180, 0, 0])
                cylinder(h=c_bot_h - c_fillet, r1=core_radius, r2=core_bot, $fn=3);
            }
            sphere(r=c_fillet, $fn=20);
        }
        
        translate([0, 0, -c_bot_h - 10])
        cylinder(h=c_top_h + c_bot_h + 20, r=spine_radius, $fn=20);
        
        cable_offset = max(spine_radius + cable_radius + 1.0, radius - (cable_radius * 4));
        
        for(angle = [0, 120, 240]) {
            rotate([0, 0, angle])
            translate([cable_offset, 0, -c_bot_h - 10])
            cylinder(h=c_top_h + c_bot_h + 20, r=cable_radius, $fn=20);
        }
    }
}

// Renders the single targeted segment
single_segment(i);
