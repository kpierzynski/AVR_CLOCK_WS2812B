base_th = 0.28*4;
wall_th = 0.28*6;
h = 6 + base_th;

D = 157+1+2;
d = 145-2;

$fn = 512;

name = "YourName";

sectors = [1,2,4,5,7,8,10,11];
big_sectors = [0,3,6,9];
guides = [ [60,big_sectors], [80,sectors] ];

module sector(h, d, a1, a2) {
    if (a2 - a1 > 180) {
        difference() {
            cylinder(h=h, d=d);
            translate([0,0,-0.5]) sector(h+1, d+1, a2-360, a1); 
        }
    } else {
        difference() {
            cylinder(h=h, d=d);
            rotate([0,0,a1]) translate([-d/2, -d/2, -0.5])
                cube([d, d/2, h+1]);
            rotate([0,0,a2]) translate([-d/2, 0, -0.5])
                cube([d, d/2, h+1]);
        }
    }
}   

module ring(D,d,th) {
    linear_extrude(height=th) difference() {
        circle(d=D);
        circle(d=d);
    }
}

module wall(D,h,th) {
    linear_extrude(height=h) difference() {
        circle(d=D);
        circle(d=D-th);
    }
}

for( guide = guides ) {
    difference() {
        union() {
            for( s = guide[1] ) 
                rotate([0,0,-2.5/2+s*30]) sector(base_th,d,0,2.5);
        }
        cylinder(d=guide[0]/100*d, h=base_th);
    }
}

translate([-60/100*d/2,0,base_th/2]) { 
    cube([1,30,base_th], center=true);
    translate([0,0,-base_th/2])
        rotate([0,0,-90])
            resize([30,0,0]) linear_extrude(height=base_th)
                text(name, valign="bottom", halign="center", size=7, font="consolas");
}

wall(D,h,wall_th);
wall(d,h,wall_th);

difference() {
    ring(D,d,base_th);
    rotate([0,0,-10]) sector(base_th, D, 0, 20);
}