/* Library to produce approximate cycloidal gears according to
 * British Standard BS 978 Part 2.  Gears use ogive teeth constructed from
 * the intersection of cylinders.
 *
 * The public interface consists of the following modules:
 *  bs978_driving_wheel: A wheel that is usually driving a pinion
 *  bs978_driven_pinion: A pinion that is always driven by a wheel
 *  bs978_sometimes: A pinion that is sometimes driven and sometimes
 *   driving (note that the standard recommends the use of involute teeth
 *   (see BS 978 Part 1) on winding wheels and motion works instead of
 *   cycloidal teeth)
 *  bs978_wheel: A general ogive-toothed wheel that exposes all parameters
 *   rather than the defaults stipulated in the standard
 *
 * The public interface consists of the following functions:
 *  bs978_pitch_diameter: Compute the pitch circle diameter
 *
 * Copyright (c) 2016, Michael Robinson
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

/* Construct a wheel tooth using the BS978 ogive form
 * T = number of teeth
 * m = module
 * f = addendum factor
 * fr = radius of curvature factor
 * w  = tooth thickness factor along pitch circle (default = pi/2)
 */
module bs978_wheel_tooth(T,m,f,fr,w=PI/2)
{
    // Compute radius of curvature center
    q=bs978_curve_center(T,m,f,fr,w);
    qp=[-q[0],q[1]];
    
    // Produce tooth tip
    intersection(){
        translate(q) cylinder(h=1,r1=m*fr,r2=m*fr);
        translate(qp) cylinder(h=1,r1=m*fr,r2=m*fr);
    }
    
    // Trace the radial flanks
    phi=atan(-q[0]/q[1]);
    theta=asin(m*fr/norm(q));
    psi=theta-phi;
    rad=sqrt(norm(q)*norm(q)-m*m*fr*fr);
    linear_extrude(1) polygon([[0,0],[rad*sin(psi),rad*cos(psi)],[-rad*sin(psi),rad*cos(psi)]],2);
}

/* Return coordinates for the center of curvature for the BS978 ogive form tooth
 * T = number of teeth
 * m = module
 * f = addendum factor
 * fr = radius of curvature factor
 * w  = tooth thickness factor along pitch circle (default = pi/2)
 */
function bs978_curve_center(T,m,f,fr,w) = let (
    // Tooth tip
    t=[0,T*m/2+f*m],
    
    // Point of tooth on pitch circle
    p=[T*m/2*sin(w*180/PI/T), T*m/2*cos(w*180/PI/T)],
    
    // Distance between tip and pitch circle point
    L=norm(t-p)/2,

    // Midpoint between tip and pitch circle point
    midpt=0.5*(t+p),
    
    // Vector to center of curvature
    v=t-p,
    Q=sqrt(m*m*fr*fr-L*L),
    v2=[-v[1]*Q/norm(v),v[0]*Q/norm(v)]) 
        m*fr > L ? v2+midpt : [0,T*m/2];
        
/* Compute the diameter of the pitch circle
 * T = number of teeth
 * m = module
 */
function bs978_pitch_diameter(T,m) = T*m;

/* Construct a general wheel using the BS978 ogive form
 * T = number of teeth
 * m = module
 * f = addendum factor
 * fr = radius of curvature factor
 * w  = tooth thickness factor along pitch circle (default = pi/2)
 * root_profile = "rounded" or "square" (default)
 */
module bs978_wheel(T,m,f,fr,w=PI/2,root_profile="square")
{
    for( i = [1:T] )
        rotate(360*i/T) bs978_wheel_tooth(T,m,f,fr,w);
    
    if( root_profile == "square" )
        cylinder(r1=T*m/2-PI/2*m,r2=T*m/2-PI/2*m);
    else
    {
        // Note: round roots are derived from Swiss standard NIHS 20-10
        // because BS978 merely says "approximately semi-circular"
        q=bs978_curve_center(T,m,f,fr,w);
        t = -q[0];
        r1 = norm(q);
        beta = 180/T - asin( m*fr / r1) + asin(t/r1);
        rho_f= (T-PI)*m * sin(beta) / (2 * (1-sin(beta)));
        
        difference()
        {   
            cylinder(r1=T*m/2-PI/2*m+rho_f,r2=T*m/2-PI*m/2+rho_f);
            for(i=[1:T])
                rotate(360*(i+0.5)/T) translate([0,T*m/2-PI/2*m+rho_f,-0.5]) cylinder(h=2,r1=rho_f,r2=rho_f);
        }
    }
}

/* BS 978 driving wheel 
 * T = number of teeth in wheel (the gear being built!)
 * m = module
 * t = number of teeth in pinion to be driven
 * root_profile = "rounded" or "square" (default)
 */
module bs978_driving_wheel(T,m,t,root_profile="square")
{
    ratio = T/t;
    
    // Note: the f and fr paramters are looked up from tables written
    // in the standard.  Each column is stipulated by the number of leaves
    // in a pinion, and each row is specified by the ratio of teeth
    // between the driving wheel and the pinion.  Columns are given in 
    // the standard for pinions with leaves of 6, 7, 8, 9, 10, 12, 14, 
    // 15, and 16.  Note that pinions with 11, 13, or more than 16 leaves
    // are not specified, presumably intending either that the values be
    // interpolated or be taken from a neighboring column.
    //
    // OpenSCAD does not handle row/column table lookup, so the following
    // is a bit of a hack; each table is a list of columns from the
    // standard's tables.  The implementation uses values from the columns
    // directly, duplicating the missing columns.  However, it does
    // interpolate *within* a column using the lookup function.
    //
    // The differences between this and any other reasonable method are
    // probably too small to matter.
    table_f=[
    [[3,1.259],[4,1.280],[5,1.293],[6,1.303],[6.5,1.307],[7,1.310],[7.5,1.313],[8,1.315],[8.5,1.318],[9,1.320],[9.5,1.321],[10,1.322],[11,1.326],[12,1.328]], // Pinion with 6 leaves
    [[3,1.335],[4,1.359],[5,1.374],[6,1.385],[6.5,1.389],[7,1.393],[7.5,1.396],[8,1.399],[8.5,1.402],[9,1.404],[9.5,1.406],[10,1.408],[11,1.411],[12,1.414]], // Pinion with 7 leaves
    [[3,1.403],[4,1.430],[5,1.447],[6,1.459],[6.5,1.464],[7,1.468],[7.5,1.471],[8,1.475],[8.5,1.478],[9,1.480],[9.5,1.482],[10,1.484],[11,1.488],[12,1.491]], // Pinion with 8 leaves
    [[3,1.465],[4,1.494],[5,1.513],[6,1.526],[6.5,1.531],[7,1.536],[7.5,1.540],[8,1.543],[8.5,1.547],[9,1.549],[9.5,1.552],[10,1.554],[11,1.558],[12,1.561]], // Pinion with 9 leaves
    [[3,1.523],[4,1.554],[5,1.574],[6,1.588],[6.5,1.594],[7,1.599],[7.5,1.603],[8,1.607],[8.5,1.610],[9,1.613],[9.5,1.616],[10,1.618],[11,1.623],[12,1.626]], // Pinion with 10 leaves
    [[3,1.523],[4,1.554],[5,1.574],[6,1.588],[6.5,1.594],[7,1.599],[7.5,1.603],[8,1.607],[8.5,1.610],[9,1.613],[9.5,1.616],[10,1.618],[11,1.623],[12,1.626]], // Pinion with 11 leaves (dup of previous)
    [[3,1.626],[4,1.661],[5,1.684],[6,1.700],[6.5,1.707],[7,1.712],[7.5,1.717],[8,1.721],[8.5,1.725],[9,1.728],[9.5,1.731],[10,1.734],[11,1.739],[12,1.743]], // Pinion with 12 leaves
    [[3,1.626],[4,1.661],[5,1.684],[6,1.700],[6.5,1.707],[7,1.712],[7.5,1.717],[8,1.721],[8.5,1.725],[9,1.728],[9.5,1.731],[10,1.734],[11,1.739],[12,1.743]], // Pinion with 13 leaves (dup of previous)
    [[3,1.718],[4,1.756],[5,1.782],[6,1.799],[6.5,1.807],[7,1.812],[7.5,1.818],[8,1.822],[8.5,1.827],[9,1.830],[9.5,1.834],[10,1.837],[11,1.842],[12,1.847]], // Pinion with 14 leaves
    [[3,1.760],[4,1.801],[5,1.827],[6,1.845],[6.5,1.853],[7,1.859],[7.5,1.864],[8,1.869],[8.5,1.874],[9,1.878],[9.5,1.881],[10,1.884],[11,1.890],[12,1.895]], // Pinion with 15 leaves
    [[3,1.801],[4,1.843],[5,1.870],[6,1.889],[6.5,1.897],[7,1.903],[7.5,1.909],[8,1.914],[8.5,1.919],[9,1.923],[9.5,1.926],[10,1.929],[11,1.935],[12,1.940]]]; // Pinion with 16 or more leaves
    
    table_fr=[
    [[3,1.855],[4,1.886],[5,1.906],[6,1.920],[6.5,1.926],[7,1.930],[7.5,1.934],[8,1.938],[8.5,1.942],[9,1.944],[9.5,1.947],[10,1.949],[11,1.954],[12,1.957]], // Pinion with 6 leaves
    [[3,1.968],[4,2.003],[5,2.025],[6,2.041],[6.5,2.048],[7,2.053],[7.5,2.058],[8,2.062],[8.5,2.066],[9,2.069],[9.5,2.072],[10,2.080],[11,2.080],[12,2.084]], // Pinion with 7 leaves
    [[3,2.068],[4,2.107],[5,2.132],[6,2.150],[6.5,2.157],[7,2.163],[7.5,2.169],[8,2.173],[8.5,2.177],[9,2.181],[9.5,2.184],[10,2.187],[11,2.193],[12,2.197]], // Pinion with 8 leaves
    [[3,2.160],[4,2.202],[5,2.230],[6,2.249],[6.5,2.257],[7,2.263],[7.5,2.269],[8,2.274],[8.5,2.279],[9,2.283],[9.5,2.287],[10,2.290],[11,2.296],[12,2.301]], // Pinion with 9 leaves
    [[3,2.244],[4,2.290],[5,2.320],[6,2.341],[6.5,2.349],[7,2.356],[7.5,2.363],[8,2.368],[8.5,2.373],[9,2.377],[9.5,2.381],[10,2.385],[11,2.391],[12,2.397]], // Pinion with 10 leaves
    [[3,2.244],[4,2.290],[5,2.320],[6,2.341],[6.5,2.349],[7,2.356],[7.5,2.363],[8,2.368],[8.5,2.373],[9,2.377],[9.5,2.381],[10,2.385],[11,2.391],[12,2.397]], // Pinion with 11 leaves (dup of previous)
    [[3,2.396],[4,2.448],[5,2.482],[6,2.505],[6.5,2.516],[7,2.523],[7.5,2.530],[8,2.536],[8.5,2.542],[9,2.547],[9.5,2.552],[10,2.556],[11,2.563],[12,2.569]], // Pinion with 12 leaves
    [[3,2.396],[4,2.448],[5,2.482],[6,2.505],[6.5,2.516],[7,2.523],[7.5,2.530],[8,2.536],[8.5,2.542],[9,2.547],[9.5,2.552],[10,2.556],[11,2.563],[12,2.569]], // Pinion with 13 leaves (dup of previous)
    [[3,2.532],[4,2.589],[5,2.626],[6,2.652],[6.5,2.662],[7,2.671],[7.5,2.679],[8,2.686],[8.5,2.692],[9,2.697],[9.5,2.703],[10,2.707],[11,2.715],[12,2.722]], // Pinion with 14 leaves
    [[3,2.594],[4,2.654],[5,2.692],[6,2.719],[6.5,2.730],[7,2.739],[7.5,2.748],[8,2.755],[8.5,2.761],[9,2.767],[9.5,2.773],[10,2.777],[11,2.785],[12,2.792]], // Pinion with 15 leaves
    [[3,2.654],[4,2.715],[5,2.756],[6,2.784],[6.5,2.795],[7,2.804],[7.5,2.813],[8,2.820],[8.5,2.827],[9,2.833],[9.5,2.839],[10,2.844],[11,2.852],[12,2.859]]]; // Pinion with 16 or more leaves
    
    // Look up the column directly based on the pinion tooth count
    column_f = t <= 16 ? table_f[t-6] : table_f[10];
    column_fr = t <= 16 ? table_fr[t-6] : table_fr[10];
    
    // Interpolate within a column based on gear ratio
    f=lookup(ratio,column_f);
    fr=lookup(ratio,column_fr);
    
    // Construct the wheel
    bs978_wheel(T,m,f,fr,PI/2,root_profile);
}

/* BS 978 driven pinion
 * T = number of teeth
 * m = module
 * root_profile = "rounded" or "square" (default)
 * profile = (default : based on tooth count), "A", "B", or "C"
 */
module bs978_driven_pinion(T,m,root_profile="square",profile=undef)
{
    if( profile == undef )
    {
        if( T <= 7 )
            bs978_wheel(T,m,0.855,1.05,1.05,root_profile); // Profile C
        else if( T <= 9 )
            bs978_wheel(T,m,0.67,0.70,1.05,root_profile); // Profile B
        else // 10 teeth and more
            bs978_wheel(T,m,0.625,0.625,1.25,root_profile); // Profile A
    }
    else if( profile == "A" )
    {
        if( T <= 10 )
            bs978_wheel(T,m,0.525,0.525,1.05,root_profile);
        else
            bs978_wheel(T,m,0.625,0.625,1.25,root_profile);
    }
    else if( profile == "B" )
    {
        if( T <= 10 )
            bs978_wheel(T,m,0.67,0.70,1.05,root_profile);
        else
            bs978_wheel(T,m,0.805,0.84,1.25,root_profile);
    }
    else
    {
        if( T <= 10 )
            bs978_wheel(T,m,0.855,1.05,1.05,root_profile);
        else
            bs978_wheel(T,m,1.05,1.25,1.25,root_profile);
    }
}

/* BS 978 sometimes driven pinion
 * T = number of teeth
 * m = module
 * root_profile = "rounded" or "square" (default)
 */
module bs978_sometimes(T,m,root_profile="square")
{
    f=lookup(T,[
        [8,1.16],
        [9,1.17],
        [10,1.19],[11,1.19],
        [12,1.20],[13,1.20],
        [14,1.22],[16,1.22],
        [17,1.24],[20,1.24],
        [21,1.26],[25,1.26],
        [26,1.27],[34,1.27],
        [35,1.29],[54,1.29],
        [55,1.31],[134,1.31],
        [135,1.32]]);
    fr=lookup(T,[
        [8,1.85],
        [9,1.87],
        [10,1.90],[11,1.90],
        [12,1.92],[13,1.92],
        [14,1.95],[16,1.95],
        [17,1.98],[20,1.98],
        [21,2.01],[25,2.01],
        [26,2.03],[34,2.03],
        [35,2.06],[54,2.06],
        [55,2.09],[134,2.09],
        [135,2.11]]);
    bs978_wheel(T,m,f,fr,1.41,root_profile);
}

/* Spokes die; intended to be used to cut spokes in a wheel
 * n = number of spkes
 * outer_diameter = outer diameter of the cut
 * inner_fraction = sets innermost diameter of cut relative to outer diameter
 * spoke_width_fraction = sets outer spoke width as fraction of outer sector
 * spoke_broadening_factor = sets inner spoke width as fraction of outer spoke width
 *
 * Example:
 * difference() {
 *   bs978_driving_wheel(48,1,10,"round");
 *   translate([0,0,-0.5]) scale([1,1,2]) spoke_die(5,bs978_pitch_diameter(48,1)*0.8);
 * }
 */
module spoke_die(n, outer_diameter, inner_fraction=0.20, spoke_width_fraction=0.05,spoke_broadening_factor=1.5)
{
    outer_radius=outer_diameter/2;
    inner_diameter=outer_diameter*inner_fraction;
    inner_radius = inner_diameter/2;
    outer_angle=360/n*spoke_width_fraction;
    spoke_outer_width=outer_diameter/2*sin(outer_angle);
    inner_angle=(spoke_outer_width*spoke_broadening_factor/inner_radius<1) ? asin(spoke_outer_width*spoke_broadening_factor/inner_radius) : 360/n;
    
    translate([0,0,0.5]) difference()
    {
        cylinder(d1=outer_diameter,d2=outer_diameter,h=1,center=true);
        
        translate([0,0,-1]) for ( i = [1:n] ){
            rotate((i-1)*360/n){
                linear_extrude(2) polygon([[inner_radius*cos(inner_angle),inner_radius*sin(inner_angle)],[1.2*outer_radius*cos(outer_angle),1.2*outer_radius*sin(outer_angle)],[inner_radius*cos(inner_angle),-inner_radius*sin(inner_angle)],[1.2*outer_radius*cos(outer_angle),-1.2*outer_radius*sin(outer_angle)]], [[0,1,3,2]], 2); 
            }
        }
        cylinder(d1=inner_diameter,d2=inner_diameter,h=2,center=true);
    }
}

$fn=100;
difference()
{
    bs978_driving_wheel(48,1,10,"round");
    translate([0,0,-0.5]) scale([1,1,2]) spoke_die(5,bs978_pitch_diameter(48,1)*0.8);
}