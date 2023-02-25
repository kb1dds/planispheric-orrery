/* This library specifies the assembly for a solar-sideral motion work
 * 
 * Copyright (c) 2023, Michael Robinson
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

use <bs978.scad>;

/* Notes:
 *
 * Key special variables:
 *  $cut_teeth = boolean: true = draw individual teeth on wheels, 
 *                         false = draw pitch circles only
 * 
 */

/** Convenience functions **/
module bs978_driving_wheel_spoked(T,m,t,spokes=5,hole_diameter=1,spoke_fraction=0.9)
{
    difference(){
        if($cut_teeth)
            bs978_driving_wheel(T,m,t,root_profile="rounded");
        else
            cylinder(d1=bs978_pitch_diameter(T,m),d2=bs978_pitch_diameter(T,m),h=1);
        translate([0,0,-0.5]) scale([1,1,2]) union() {
           spoke_die(spokes,bs978_pitch_diameter(T,m)*spoke_fraction);
           cylinder(h=2,d1=hole_diameter,d2=hole_diameter);
        }
    }
}

module bs978_pinion_drilled(t,m,hole_diameter){
    difference(){
        bs978_driven_pinion(t,m,root_profile="rounded");
        translate([0,0,-0.5]) cylinder(h=2,d1=hole_diameter,d2=hole_diameter);
    }
}

/* Four wheel train calculation function
 * Origin is center of first wheel in train
 * End of train is on x-axis, (train_length,0)
 * Returns: list of 4: each element is a list of 4 (also):
 *   x,y coordinates of center
 *   rotation angle of wheel
 *   rotation angle of pinion
 * Assumptions:
 *  First wheel is a bare pinion
 *  Last wheel has no pinion
 */
function four_train_data(initial_angle,train_length,m,t1,T2,t2,T3,t3,T4) =
    let( // Wheel separations
        r1=(bs978_pitch_diameter(t1,m)+bs978_pitch_diameter(T2,m))/2, 
        r2=(bs978_pitch_diameter(t2,m)+bs978_pitch_diameter(T3,m))/2,
        r3=(bs978_pitch_diameter(t3,m)+bs978_pitch_diameter(T4,m))/2,

        // Intermediate calculations
        R=sqrt(r1*r1*sin(initial_angle)*sin(initial_angle)+(train_length-r1*cos(initial_angle))*(train_length-r1*cos(initial_angle))),
        alpha=asin(r1/R*sin(initial_angle)),
        phi=acos((R*R+r3*r3-r2*r2)/(2*r3*R)),
        
        // Wheel centers
        x1=0,
        y1=0,
    
        x2=r1*cos(initial_angle),
        y2=r1*sin(initial_angle),
    
        x3=train_length-r3*cos(alpha+phi),
        y3=r3*sin(alpha+phi),
    
        x4=train_length,
        y4=0,
        
        // Angle intermediate calculation        
        psi = atan2(y3-y2,x3-x2))
        [[x1,y1, undef, 90+initial_angle],
         [x2,y2, initial_angle+90+180/T2, 90+psi],
         [x3,y3, 90+psi+180/T3, 90-alpha-phi-180/t3],
         [x4,y4, 90-alpha-phi, undef]
        ];
        
 /* Four wheel train arbors
 * Origin is center of first wheel in train
 * End of train is on x-axis, (train_length,0)
 */
module four_train_arbors(initial_angle,train_length,m,t1,T2,t2,T3,t3,T4,hole_diameter=2){
    /* Determine coordinates and angles */
    data = four_train_data(initial_angle,train_length,m,t1,T2,t2,T3,t3,T4);
    
    /* Construct arbors */
    for( d=data ){
        translate([d.x,d.y,0]) cylinder(h=2,d=hole_diameter,center=true);
    }
}
        
/* Four wheel train 
 * Origin is center of first wheel in train
 * End of train is on x-axis, (train_length,0)
 */
module four_train(initial_angle,train_length,m,t1,T2,t2,T3,t3,T4,spokes=5,hole_diameter=2, spoke_fraction=0.75){
    
    /* Determine coordinates and angles */
    data = four_train_data(initial_angle,train_length,m,t1,T2,t2,T3,t3,T4);
    
    /* First wheel in train is a bare pinion */
    translate([data[0].x,data[0].y,0]) rotate(data[0][3]) bs978_pinion_drilled(t1,m,hole_diameter);
    
    /* Second wheel and pinion */
    translate([data[1].x,data[1].y,0]) rotate(data[1][2]) bs978_driving_wheel_spoked(T2,m,t1,spokes,hole_diameter,spoke_fraction);
    translate([data[1].x,data[1].y,1]) rotate(data[1][3]) bs978_pinion_drilled(t2,m,hole_diameter);
    
    /* Third wheel and pinion */
    translate([data[2].x,data[2].y,1]) rotate(data[2][2]) bs978_driving_wheel_spoked(T3,m,t2,spokes,hole_diameter,spoke_fraction);
    translate([data[2].x,data[2].y,0]) rotate(data[2][3]) bs978_pinion_drilled(t3,m,hole_diameter);
     
    /* Four wheel has no pinion */
    translate([data[3].x,data[3].y,0]) rotate(data[3][2]) bs978_driving_wheel_spoked(T4,m,t3,spokes,hole_diameter,spoke_fraction);
}

/* Solar train: drive arbor at origin, oriented positive z-axis */
module solar_train(){
    union(){
        four_train(initial_angle=80,train_length=50,m=1,t1=12,T2=24,t2=12,T3=36,t3=12,T4=48);
        scale([1,1,4]) four_train_arbors(initial_angle=80,train_length=50,m=1,t1=12,T2=24,t2=12,T3=36,t3=12,T4=48);
    }
}

/* Sidereal train: drive arbor at origin, oriented positive z-axis */
module sidereal_train(){
    union(){
        four_train(initial_angle=70,train_length=50,m=1.25,t1=8,T2=23,t2=10,T3=27,t3=12,T4=37);
        scale([1,1,4]) four_train_arbors(initial_angle=70,train_length=50,m=1.25,t1=8,T2=23,t2=10,T3=27,t3=12,T4=37);
    }
}

/* Motion work: drive arbor at origin, oriented positive z-axis */
module motion_work(){
    translate([0,0,2.5]){
        /* Solar train */
        #mirror([0,1,0]) mirror([0,0,1]) translate([0,0,1]) solar_train();

        /* Sidereal train */
        sidereal_train();
    };
}

$cut_teeth=true;
$fn=20;

//projection(cut=true)
motion_work();