////////////////////////////////////////////////////////
// 
// Ivan the Robot Arm
// 
// Les Hall
// started Sun Feb 19 2017
// 
// 
// parts list:  
//      order filament
//          Aqua Translucent PETG
//      axles
//          hardware replaced with printable design
//      fasteners
//          16 x motor plate 3mm screws for metal
//          8 x top_disc(); 3mm screws for plastic
//          8 x base_plate(); 3mm screws for plastic (same as above)
//          8 x stand 3mm screws for plastic, long for base mounting
//      electronics parts
//          Circuit Playground or other Arduino
//          8 x L293D or equivalent
//          2 x CMOS hex inverter
//          2 x full size breadboard
//          2 x 2.1mm female power entry jack terminal block
//          1 x 2.1mm male power entry jack terminal block
//          2 x 100 uF electrolytic capacitor
//          2 x 0.1 uF ceramic capacitor
//          20 x 100 kOhm resistor
//          4 x 0.1" headers for motor cables
//          flexible stranded wire for magnetic reed switches
//          24 ga. solid hook-up wire
//          7 x magnetic reed switches
//          14 x magnets 3/8" x 1/8" disc
//          LM7805 5V regulator
//          LM317T trickle charge circuit (regulator, R's, C's)
//          large 12V sealed lead-acid battery
// 
//  features and to do list:
//      software
//          Arduino / Circuit Playground
//          stepper control written
//          kinematics equations
//          pattern generator
//          serial port comm
//          Processing control program
//      gears
//          add D key flats to motor shaft gears (make optional)
//          or maybe leave off to act as safety in event of over-torque
//          recess fixed driven hinge gear screws and motor mounting screws
//      Grab-Tor
//          add retraction string details to end of arm
//          add screw holes for attaching Grab-Tor and Grab-Tor shaft
//          hollow out Grab-Tor shaft
//          attach Grab-Tor shaft with screw holes
//      axle bearings
//          examine spacers / gaps so bearings work properly
//      add washers and screws - Skip's idea
//          #6 or #8, thread cutting screws, McMaster Carr 
//      add position feedback for arm motors
//          electronic?
//              circuit playground?
//          potentiometer?
//      make threaded rods into large diameter 3D printed axles
//          reduces parts count - fewer items on parts list
//          increases printable content
// 
//      angular feedback
//          place accelerometer (Circuit Playground) after final hinge
//          to measure ph_ + beta.  Then algebraically substitute into 
//          control equations to determine the values of ph_and beta, 
//          which gives the angular values of all arm motors.  Calibrate
//          by adjusting until arm is fully vertical, which means 
//          ph_ = -beta = 0.  
// 



// include files
include <MCAD/involute_gears.scad>
use <Grab-Tor.scad>



// selection parameter
selection = 0;  // [0-15] (see below for assignments)
gearsVisible = false;  // gear parameters
grabTorPresent = false;  // presence or lack of presence of Grab-Tor
forkPresent = true;  // presence or lack of presence of fork
circPitch = 8;  // tooth length in mm 
                // scales the size of everything
teeth1 = 16;  // sun gear teeth
teeth2 = 16;  // planet gear teeth
teeth4 = 16;  // motor drive gear
tMax = 4;  // number of planet gears

// external part parameters
nozzle = 0.8;  // diameter of nozzles
bearing608 = [7, 22 + nozzle, 8];  // height, OD, ID
magnetSize = [1/8 * 25.4, 3/8* 25.4];
reedSwitchSize = [1/2 * 25.4, 1/8 * 25.4];

// motor parameters
motorPlateSize = [43.2, 43.2, 4];
plate = motorPlateSize[2];
motorSize = [1, 1, 1] * 43.2;
motorShaftDiameter = 5.2;
motorHoleDiameter = 3;
motorHoleSpacing = 31;
motorCenterHoleDiameter = 22;

// Ivan Arm parameters
numSections = 3;
numStandGaps = 8;
GrabTorID = 25.4;  // actuator inside diameter
GrabTorRingHeight = 12;  // find the actual value

// color parameters
// 
// assign values to schemes
colorSchemeUSA = 1;
colorSchemeDanger = 2;
colorSchemeAqua = 3;
//
// select color scheme
colorScheme = colorSchemeAqua;
//
// define color schemes
colorSolid = "Aqua";
colorGears = 
    colorScheme == colorSchemeUSA ? "White" : 
    colorScheme == colorSchemeDanger ? "Yellow" : 
    colorSolid;
colorSides = 
    colorScheme == colorSchemeUSA ? "Blue" : 
    colorScheme == colorSchemeDanger ? "Black" :
    colorSolid;
colorMounts = 
    colorScheme == colorSchemeUSA ? "Red" : 
    colorScheme == colorSchemeDanger ? "Black" : 
    colorSolid;
colorStand = 
    colorScheme == colorSchemeUSA ? "Red" : 
    colorScheme == colorSchemeDanger ? "Black" : 
    colorSolid;
colorDisc = 
    colorScheme == colorSchemeUSA ? "Blue" : 
    colorScheme == colorSchemeDanger ? "Yellow" : 
    colorSolid;
colorTool = 
    colorScheme == colorSchemeUSA ? "Blue" : 
    colorScheme == colorSchemeDanger ? "Yellow" : 
    colorSolid;
colorMotor = "Silver";
colorWood = "Tan";
colorBowl = "White";



// general parameters
fastenerDiameter = 3;
$fn = 32;


// function definitions
function pitch(mm) = mm * 180 / PI;
function radius(cp, t) = cp * t / (2*PI);
function teeth(cp, r) = 2*PI * r / cp;



// function calculations
dist = 4 * circPitch;  // spacing distance
radius1 = radius(circPitch, teeth1);
radius2 = radius(circPitch, teeth2);
distance12 = radius1 + radius2;
radius3 = radius1 + 2*radius2;
teeth3 = teeth(circPitch, radius3);
gear3Diameter = 2.35*radius3;
outerDiameter = 2.5*radius3;
radius4 = radius(circPitch, teeth4);
radius5 = (motorSize[1]/2 + plate + dist) - radius4;
teeth5 = teeth(circPitch, radius5);


// parameter calculations
height = circPitch;  // widely used parameter
gearHeight = 2*bearing608[0];  // height of gears
motorGearHeight = circPitch;  // thickness of motor gear
edge = circPitch/2;  // height of base frames
armWidth = motorSize[1] + 2*plate;  // width of arms
postDiameter = bearing608[2] + nozzle;  // diameter of posts
clearance = circPitch / 10;  // spacing around posts
offset = 0;  // extra room between gears 1 and 2
basePostDiameter = 2*circPitch;  // diameter of posts separating base sides

// link parameters
linkDiameter = circPitch + bearing608[1];
linkCenterThickness = (motorSize[2] + 2*plate)/2;
linkSideThickness = (motorSize[2] + 2*plate)/4;

deltaX = outerDiameter - gear3Diameter;  // difference in diameters

// more calculations
armLength = motorSize[1] + 2*plate + 2*dist; // length of arms

// belt dimensions
beltThickness = circPitch / 2;
beltWidth = 2 * circPitch;
beltDiameter = radius1[1] + beltThickness;
beltLength = armLength + 2*beltThickness;

// printouts
echo("radius1", radius1);
echo("radius2", radius3);
echo("radius3", radius3);
echo("radius4", radius4);
echo("radius5", radius5);
echo("outerDiameter", outerDiameter);
echo("teeth1", teeth1);
echo("teeth2", teeth2);
echo("teeth3", teeth3);
echo("teeth4", teeth4);
echo("teeth5", teeth5);
echo("armLength", armLength);
echo("gear box thickness", gearHeight + 2*edge);

// animation variables - FPS: 2, Steps: 90
// Play "tag" with the robot.  Move the toolhead to touch 
// a positioned ball.  Use some simple equations 
// (4 trig expressoins) to command the robot's motion.
// 
// first position the ball
randomize = 1;
sinusoidal = 2;
feeding = 3;
pattern = feeding;
th_ = pattern == randomize ? 
    rands(-180, 180, 1)[0] : 
    pattern == sinusoidal ?
    180 * cos(360 * $t) : 
    -90 + 90 * (1 + cos(360 * $t))/2;
ph_ = pattern == randomize ? 
    rands(-20, 120, 1)[0] : 
    pattern == sinusoidal ?
    50 + 70 * cos(360 * $t) : 
    120 - 60 * pow((1+cos(360 * $t))/2, 1/2);
r_ = pattern == randomize ? 
    rands(armLength, armLength*3, 1)[0] : 
    pattern == sinusoidal ?
    armLength*2 + armLength * (1 + sin(2 * 360 * $t))/2 : 
    2.5*armLength + 1/2*armLength * cos(360 * $t);
// 
// the primary equation of motion:  
beta = acos( (r_/3) / (armLength) ) * 4/PI;
gamma = beta + 15;  // angle for Grab-Tor opening and closure
//
// command the robot's shape
theta = [90 + th_,  ph_ - beta, beta, beta];
echo("theta", theta);
// 
// and draw the red ball
if (selection == 0)
    color("Magenta")
    rotate(th_)
    rotate([ph_, 0, 0])
    translate([0, 0, r_])
    sphere(25/2);
// 
// the rest of this file is dedicated to drawing Ivan.  
// note:  use animation from the view pulldown.  
// 



// printables
planetary_gear_set(selection);
module planetary_gear_set(sel) {
    
    if (sel == 0) {
        Ivan();                 // my robot Ivan
    } else if (sel == 1) {
        assembly(0, 0, 0, 1, 0);
    } else if (sel == 2) {
        base_plate(0);  // no arms, yes supports
    } else if (sel == 3) {
        gear1(0);                // sun gear
    } else if (sel == 4) {
        gear2(0);  // one planet gear (print six)
    } else if (sel == 5) {
        rotate(45)
        gear3();         // outer gear, no arm
    } else if (sel == 6) {
        rotate([0, 180, 0])
        stand();              // footing of arm
    } else if (sel == 7) {
        link(0, 0);    // base link
    } else if (sel == 8) {
        link(1, 0);    // arm link
    } else if (sel == 9) {
        link(2, 0);    // tool link
    } else if (sel == 10) {
        top_disc(deltaX, 0);    // base of arm
    } else if (sel == 11) {
        gear4(0);  // motor drive gear
    } else if (sel == 12) {
        gear5(0);  // driven hinge gear
    } else if (sel == 13) {
        basePost();
    } else if (sel == 14) {
        Grab_Tor_print(GrabTorID);  // actuator print tray
    } else if (sel == 15) {
        Grab_Tor(GrabTorID, 0);  // actuator
    }
}



// my robot Ivan
module Ivan() {
    
    z0 = armLength - dist + edge + gearHeight/2;  // raises ball to center
    
    translate([0, 0, -z0])
    union() {
        
        rotate(180 - 180 / numStandGaps)
        union() {
            
            // the horizontal base of the arm
            assembly(0, 0, theta[0], 0, 0);
            // the horizontal base motor
            color(colorMotor)
            translate([0, 0, -motorSize[2]/2 - 
                gearHeight/2 - 3*edge])
            cube(motorSize, center = true);
    
            // the verticals of the arm
            if (!gearsVisible) {
                translate([0, 0, 
                    armLength - dist +
                        gearHeight/2 + height])
                rotate(theta[0])
                rotate(90 + 180/numStandGaps)
                links(1);
            }
        }
        
        // the stand for attaching robot arm
        // and the wooden base as well
        translate([0, 0, -gearHeight/2 - edge - clearance])
        union() {
            
            // the stand
            color(colorStand)
            stand();
            
            // the wooden base and bowl
            translate([0, 0, -(motorSize[2] + edge)])
            union() {
                
                // the wooden base
                color(colorWood)
                translate([-75, 0, -3/4 * 25.4])
                wooden_base();
                
                // the bowl
                color(colorBowl)
                translate([-200, 0, 0])
                bowl();
            }
        }
    }
}



module bowl() {
    
    translate([0, 0, 25])
    difference() {
        
        // outer shape
        union() {
            
            // outer tapered cylinder
            cylinder(h = 50, d1 = 25, d2 = 150);
            
            // wide base
            cylinder(h = 40, d1 = 100, d2 = 0);
        }
        
        // inner tapered cylinder
        translate([0, 0, 5])
        cylinder(h = 50, d1 = 25, d2 = 150);
    }
}



module wooden_base() {
    
    // the main shape
    translate([0, 0, 3/4 * 25.4])
    cube([350, 250, , 3/8 * 25.4], center = true);
    
    // the footing slats
    for (side = [-1:2:1])
        translate([125 * side, 0, 3/8 * 25.4])
        cube([25, 250, 3/8 * 25.4], center = true);
}



module links(i) { 

    type = i == 1 ? 0 : i <= numSections ? 1 : 2;

    rotate([0, 90, 0]) {
    
        // the link
        link(type, 1);
        
        // the motor's drive gear
        if (i < numSections+1)
            color(colorGears)
            translate([0, 0, -(motorSize[2]/2 + plate)])
            translate([i == 1 ? 
                (motorSize[1]/2 + plate) + dist : 
                -armLength/2, 0, 0])
            translate([0, 0, -motorGearHeight/2 - clearance])
            rotate([0, 0, -theta[i] * teeth5/teeth4])
            rotate(180/teeth4)
            gear4(0);
        
        // the fixed driven wheel at hinge point
        if (i > 1)
            color(colorGears)
            translate([0, 0, -(motorSize[2]/2 + plate)])
            translate([0, 0, -motorGearHeight/2 - clearance])
            gear5(0);
        
        // the end treatment (mounting assembly)
        if (i == numSections+1) {  // if end section
            
            // the toolhead
            translate([-dist-linkSideThickness, 0, 0])
            rotate([0, 0, theta[i]])
            rotate([0, -90, 0])
            if (grabTorPresent) {
               
                // the extender tube
                color(colorTool)
                translate([0, 0, (armLength - dist - 
                    linkSideThickness)/2])
                tube(H = armLength - dist - linkSideThickness, 
                    OD = GrabTorID, ID = GrabTorID-10);
                
                // the Grab-Tor!
                color(colorTool)
                translate([0, 0, armLength - dist - 
                    linkSideThickness - GrabTorRingHeight/2])
                rotate([0, 180, 0])
                Grab_Tor(GrabTorID, gamma);  // actuator
            
            } else if (forkPresent) {
                fork();
            }
        }
    }
    
    // draw the next assembly
    if (i <= numSections)
        translate([0, 0, i > 1 ? armLength : 0])
        rotate([theta[i], 0, 0])
        links(i+1);
}




module link(type, motor) {
        
    translate([type > 0 ? -armLength : 0, 0, 0]) {
        
        // the central section
        if (type != 2)
        color(colorSides)
        difference() {
            
            union() {
                
                // center
                cylinder(h = linkCenterThickness, 
                    d = linkDiameter, 
                    center = true, $fn = 4*$fn);
                
                // arm
                translate([dist/2, 0, 0])
                cube([dist, linkDiameter, 
                    linkCenterThickness], 
                    center = true);
            }
            
            // drill axle hole
            translate([0, 0, 1])
            cylinder(h = linkCenterThickness + 2, 
                d = bearing608[0] + 2*clearance, 
                center = true);
        
            // make space for bearings
            for (side = [-1:2:1])
                translate([0, 0, side * linkCenterThickness/2])
                cylinder(h = 2*bearing608[0], 
                    d = bearing608[1], 
                    center = true, $fn = 4*$fn);
        }
        
        // the two side sections
        if (type != 0) {
                
            color(colorSides)
            translate([armLength, 0, 0])
            difference() {
                
                union() {
                    
                    // sides
                    for(side = [-1:2:1]) {
                        
                        rotate([90 + 90 * side])
                        translate([0, 0, 
                            linkCenterThickness/2])
                        union() {
                            
                            // side
                            cylinder(h = linkSideThickness, 
                                d = linkDiameter, 
                                $fn = 4*$fn);
                            
                            // arm
                            translate([-dist/2, 0, 
                                linkSideThickness/2])
                            cube([dist, linkDiameter, 
                                linkSideThickness], 
                                center = true);
                        }
                    }
                }
                
                // drill axle hole
                for(side = [-1:2:1]) {
                    translate([0, 0, 
                        side * (linkCenterThickness/2 + 
                        linkSideThickness/2)])
                    cylinder(h = 4*linkSideThickness, 
                        d = bearing608[0], 
                        center = true);
                }
                
                // drill fastener holes
                translate([0, 0, 
                    -(linkCenterThickness/2 + 
                    linkSideThickness/2)])
                for (t = [0:3])
                    rotate(360 * (t+1/2)/4)
                    translate([linkDiameter*3/8, 0, 0])
                    cylinder(h = 2*linkSideThickness, 
                        d = fastenerDiameter, 
                    center = true);
            }
        }
        
        // flat on tool piece
        if (type == 2) {
            
            color(colorSides)
            translate([
                motorSize[1] + 2*plate + 
                dist - linkSideThickness/2, 
                0, 0])
            cube([linkSideThickness, 
                linkDiameter, 
                motorSize[1] + 2*plate 
                ], 
                center = true);
        }
        
        // draw the motor and motor housing
        if (type != 2) {
            
            translate([armLength/2, 0, 0])
            rotate([0, 90, 0])
            motor(1, motor);
        }
    }
    
    // draw mounting flanges on bottom piece
    if (type == 0) {
        
        // draw the plate
        color(colorMounts)
        translate([dist + motorSize[1] + plate*3/2, 0, 0])
        difference() {
            
            // the plate
            rotate([0, 90, 0])
            cube([motorPlateSize[0], 3/2 * motorPlateSize[1], plate] +
                [2, 1, 0] * plate, 
                center = true);
        
            // drill the holes
            rotate([0, 180, 0])
            drill_base_plate_holes(plate);
        }
    }
}



module drill_base_plate_holes(thick) {
    
    // drill the holes
    for (x = [-1:2:1], y = [-1:2:1]) {
        
        translate([0, 
            x * 11/16*motorPlateSize[0], 
            y * 3/8*motorPlateSize[1]])
        rotate([0, 90, 0])
        cylinder(h = 4*thick, 
            d = fastenerDiameter, 
            center = true);
    }
    
    // drill the countersinks
    for (x = [-1:2:1], y = [-1:2:1]) {
        
        translate([thick/2, 
            x * 11/16*motorPlateSize[0], 
            y * 3/8*motorPlateSize[1]])
        rotate([0, 90, 0])
        cylinder(h = fastenerDiameter, 
            d = 2*fastenerDiameter, 
            center = true);
    }
}




// the assembled parts for one section (not printable)
module assembly(arms, motor, theta, phi, s) {
    
    // lower base
    color(colorSides)
    translate([0, 0, -(gearHeight/2 + edge/2 + clearance)])
    rotate([0, 0, 180])
    rotate([phi, 0, 0])
    base_plate(s);
        
    // upper base
    if (gearsVisible == false)
    color(colorSides)
    translate([0, 0, gearHeight/2 + edge/2 + clearance])
    rotate([180, 0, 0])
    rotate([0, 0, 180])
    rotate([phi, 0, 0])
    base_plate(s);
    
    // base separating posts
    color(colorSides)
    for(t = [0:tMax-1])
        rotate(360 * (t+1/2)/tMax)
        translate([distance12 + offset + radius2/3, 0, 0])
        basePost();
    
    // inner gear
    color(colorGears)
    rotate(-teeth3/teeth1*theta)
    rotate(180/teeth1)
    gear1(s);

    // planetary gears
    color(colorGears)
    for(t = [0:tMax-1])
        rotate(360*t/tMax)
        translate([distance12 + offset, 0, 0])
        rotate(teeth3/teeth2 * theta)
        gear2(s);
    
    // outer gear
    rotate(theta + 180/numStandGaps)
    gear3();
    
    // top
    if ( (s == 0) && (gearsVisible == false) )
        translate([0, 0, gearHeight/2 + 
            3/2*edge + clearance])
        rotate(theta)
        top_disc(deltaX, 0);

    if (s > 0)
        rotate(theta)
        translate([dist + motorSize[1]/2, 0, 0])
        rotate([0, 90, 0])
        motor(1, 1);
}



// mount for whole robot arm
// attaches to lowest motor mount
module stand() {
    
    h = motorSize[2] + edge;
    r = outerDiameter/2;
    
    difference() {
        
        // the main shape
        translate([0, 0, -h/2 - edge/2])
        cylinder(h = h + edge, r = r, 
            center = true, $fn = 4*$fn);
            
        // printability waffle pattern, concentric rings
        ringNum = 4;
        for (k = [2:ringNum-1])
            tube(H = edge/2, 
                OD = outerDiameter*k/ringNum, 
                ID = outerDiameter*k/ringNum - 3);
        
        // printability waffle pattern, radial lines
        lineNum = 4;
        for (k = [0:lineNum-1])
            rotate(180 * (k+1/2)/lineNum)
            cube([outerDiameter, 3, edge/2], center = true);
        
        // subtract openings
        for (t = [0:numStandGaps-1])
            rotate(360 * t/numStandGaps)
            translate([r, 0, -1 -h*3/4 - edge])
            cube([2*r, PI*r/numStandGaps, h/2 + edge], 
                center = true);
        
        // remove triangle tips
        translate([0, 0, -h/2 - edge*3/2])
        cylinder(h = h + edge, r = r - edge*2, 
            center = true, $fn = 4*$fn);
    
        // add drill holes to bottom
        for (t = [0:numStandGaps-1])
            rotate(360 * (t+1/2)/numStandGaps)
            translate([r - edge, 0, 
                -h - edge])
            cylinder(h = edge*2, 
                d = fastenerDiameter, 
                center = true);
        
        // add drill holes for motor mount to top
        for (t = [0:tMax-1])
            rotate(360 * (t+1/2)/tMax)
            translate([motorHoleSpacing/sqrt(2), 0, 
                -edge/2])
            cylinder(h = edge*2, 
                d = fastenerDiameter, 
                center = true);
     
        // add center mounting drill hole
        translate([0, 0, -edge/2])
        cylinder(h = edge*2, 
            d = motorCenterHoleDiameter, 
            center = true);
        
        // add spacer holes
        for (t = [0:tMax-1])
            rotate(360 * (t+1/2)/tMax)
            translate([distance12 + offset + radius2/3, 
                0, -edge/2])
            cylinder(h = edge*2, 
                d = fastenerDiameter, 
                center = true);
        
        // cut holes for screws attaching bottom_disc()
        for (t = [0:numStandGaps-1])
            rotate(360 * t/numStandGaps)
            translate([(gear3Diameter + 2*deltaX)/2, 0, -edge/2])
            cylinder(h = 0.5 * 25.4, 
                d = 0.075 * 25.4, 
                center = true);
    }
    
    // add reed switch holders to top of stand
    for (t = [0:0])
        rotate(360 * t/numStandGaps)
        translate([r, 0, edge - clearance - 2*reedSwitchSize[1]])
        rotate([90, 0, 0])
        difference() {
            
            // outer shape
            cube([3*reedSwitchSize[1], 
                2*reedSwitchSize[1],
                reedSwitchSize[0]], 
                center = true);
            
            // inner shape
            translate([reedSwitchSize[1]*2/3, 0, 0])
            cylinder(h = reedSwitchSize[0]+2, 
                d = reedSwitchSize[1], 
                center = true);
        }
}



// posts to separate base sides
module basePost() {
    
    tube(gearHeight + 2*clearance, basePostDiameter, fastenerDiameter);
}



// base frames
module base_plate(s) {
    
    // sun gear center
    color(colorSides) {
        
        difference() {
            
            // disc shape
            tube(edge, 
                outerDiameter, 
                motorCenterHoleDiameter);
            
            // spacer holes
            for (t = [0:tMax-1])
                rotate(360 * (t+1/2)/tMax)
                translate([distance12 + offset + radius2/3, 
                    0, 0])
                cylinder(h = edge*4, 
                    d = fastenerDiameter, 
                    center = true);
        
            // fastener recesses for spacer holes
            for (t = [0:tMax-1])
                rotate(360 * (t+1/2)/tMax)
                translate([distance12 + offset + radius2/3, 
                    0, -edge/2-fastenerDiameter/2])
                cylinder(h = fastenerDiameter, 
                    d = 2*fastenerDiameter);
            
            // motor holes
            for (t = [0:tMax-1])
                rotate(360 * (t+1/2)/tMax)
                translate([motorHoleSpacing/sqrt(2), 
                    0, 0])
                union() {
                    
                    // the hole
                    cylinder(h = edge*2, 
                        d = fastenerDiameter, 
                        center = true);
                    
                    // the countersink
                    cylinder(h = edge*2, 
                        d = 2*fastenerDiameter);
                }
                
            // printability waffle pattern, concentric rings
            ringNum = 4;
            translate([0, 0, -edge/2])
            for (k = [2:ringNum-1])
                tube(H = edge/2, 
                    OD = outerDiameter*k/ringNum, 
                    ID = outerDiameter*k/ringNum - 3);
            
            // printability waffle pattern, radial lines
            lineNum = 4;
            translate([0, 0, -edge/2])
            for (k = [0:lineNum-1])
                rotate(180 * (k+1/2)/lineNum)
                cube([outerDiameter, 3, edge/2], center = true);
        }
        
        // gear2 posts
        translate([0, 0, edge/2])
        for (t = [0:tMax-1])
            rotate(360 * t/tMax)
            translate([distance12 + offset, 0, bearing608[0]/4])
            tube(gearHeight/2, postDiameter, 0);
    }
}



// sun gear
module gear1(s) {
    
    // the gear itself
    translate([0, 0, -gearHeight/2])
    gear (
        number_of_teeth = teeth1, 
        circular_pitch = pitch(circPitch), 
        backlash = 0.5, 
        clearance = 0.5, 
        bore_diameter = motorShaftDiameter,  
        hub_diameter = 0, 
        rim_width = 0, 
        gear_thickness = gearHeight, 
        rim_thickness = gearHeight, 
        hub_thickness = gearHeight, 
        circles = 0);
    
    // make "D" flat
    translate([motorShaftDiameter/2, 0, 0])
    cube([1, 5, gearHeight], center = true);
}



// planet gears
module gear2(s) {
    
    translate([0, 0, -gearHeight/2])
    difference() {
        
        gear (
            number_of_teeth = teeth2, 
            circular_pitch = pitch(circPitch), 
            backlash = 0.5, 
            clearance = 0.5, 
            bore_diameter = bearing608[1], 
            hub_diameter = 0, 
            rim_width = 0, 
            gear_thickness = gearHeight, 
            rim_thickness = gearHeight, 
            hub_thickness = gearHeight, 
            circles = 0);
        
        cylinder(h = gearHeight, 
            d = fastenerDiameter);
    }
    
    // central disk
    tube(gearHeight - 2*bearing608[0], 
        bearing608[1],
        postDiameter + 2*clearance);
}



// annular gear (outer gear)
module gear3() {
    
    difference() {
        
        color(colorGears)
        cylinder(h = gearHeight, 
            d = gear3Diameter, 
            center=true, $fn = 4*$fn);
        
        color(colorGears)
        translate([0, 0, -gearHeight/2])
        gear (
            number_of_teeth = teeth3, 
            circular_pitch = pitch(circPitch), 
            backlash = -0.5, 
            clearance = -0.5, 
            //pressure_angle = 30, 
            bore_diameter = 0, 
            hub_diameter = 100, 
            rim_width = 0, 
            gear_thickness = gearHeight+1, 
            rim_thickness = gearHeight+1, 
            hub_thickness = gearHeight+1, 
            circles = 0);
    }
    
    // the support struts
    for (i = [0:numStandGaps-1]) {

        color(colorGears)
        rotate(360 * i/numStandGaps)
        translate([-deltaX/2, 0, 0])
        translate([gear3Diameter/2 + 
            deltaX/2, 0, 0]) {
            
            // side extender
            cube([deltaX, deltaX, gearHeight], 
                center = true);
            
            // post
            difference() {
                
                // post
                translate([deltaX + (i == 0 ? magnetSize[1]/2 : 0), 
                    0, edge/2+clearance/2])
                cube([(i == 0 ? magnetSize[1] : 0) +  deltaX, deltaX, 
                    gearHeight + edge + clearance], 
                    center = true);
                
                // holes
                translate([deltaX, 0, 
                    gearHeight])
                cylinder(h = 2*height, 
                    d = fastenerDiameter, 
                    center = true);
                
               // magnet sandwich
                // magnets held in place by each other's proximity
                // placed in round indendations in the struts
                if (i == 0)
                    for (side = [-1:2:1])
                        translate([deltaX+magnetSize[1]*3/4, side*(deltaX/2 + magnetSize[0]/2), 
                            -gearHeight/2 + magnetSize[1]/2])
                        rotate([90, 0, 0])
                        cylinder(h = 2*magnetSize[0], 
                            d = magnetSize[1] + 0*clearance, 
                            center = true);
            }
        }
    }        
}



module top_disc(deltaX, s) {
    
    rotate(-180 / numStandGaps) {
        
        difference() {
            
            // disc
            color(colorDisc)
            cylinder(h = edge, 
                d = gear3Diameter + 
                    4*deltaX, 
                center = true, $fn = 4*$fn);
            
            // drill holes to attch to gear3's support rods
            for (t = [0:numStandGaps-1]) {
                rotate(360 * t/numStandGaps)
                translate([radius3 + 2*deltaX, 0, 0])
                cylinder(h = 2*edge, 
                    d = fastenerDiameter, 
                    center = true);
            }
            
            // drill the base plate holes
            rotate(-360/numStandGaps)
            rotate([0, 90, 0])
            drill_base_plate_holes(edge);
        }
    }
}

 

// motor drive gear
module gear4(s) {
    
    difference() {
        
        // the gear itself
        translate([0, 0, -motorGearHeight/2])
        gear (
            number_of_teeth = teeth4, 
            circular_pitch = pitch(circPitch), 
            bore_diameter = motorShaftDiameter + nozzle, 
            hub_diameter = radius4, 
            rim_width = 0, 
            gear_thickness = motorGearHeight, 
            rim_thickness = motorGearHeight + clearance, 
            hub_thickness = motorGearHeight, 
            circles = 0);
        
        // drill marking hole
        translate([-radius4/2, 0, 0])
        cylinder(h = 2*motorGearHeight, 
            d = fastenerDiameter, 
            center = true);
    }
}



// hinge driven gear
module gear5(s) {
    
    // make gear and subtract half of it
    difference() {
        
        // the gear itself
        rotate(180)
        translate([0, 0, -motorGearHeight/2])
        gear (
            number_of_teeth = teeth5, 
            circular_pitch = pitch(circPitch), 
            bore_diameter = bearing608[0] + nozzle + 2*clearance, 
            hub_diameter = radius5/4, 
            rim_width = radius5/4, 
            gear_thickness = motorGearHeight, 
            rim_thickness = motorGearHeight + clearance, 
            hub_thickness = motorGearHeight, 
            circles = 0);
    
        // the removed part
        translate([0, 0, -motorGearHeight/2])
        difference() {
            
            // the notch
            translate([-(radius5 + circPitch), 0, 0])
            scale([1, 2/3, 1])
            rotate(45)
            cube([
                sqrt(2)*(radius5 + circPitch),
                sqrt(2)*(radius5 + circPitch),
                6*motorGearHeight], 
                center = true);
            
            // remove the hub from the notch, 
            // effectively leaving the hub on the gear
            cylinder(h = 8*motorGearHeight, 
                d = radius5*3/2);
        }
        
        // drill fastener holes
        for (t = [0:3])
            rotate(360 * t/4)
            translate([linkDiameter*3/8, 0, 0])
            cylinder(h = 2*motorGearHeight, 
                d = fastenerDiameter, 
                center = true);
    }
}



module NEMA_plate() {
    
    difference() {
        
        // the plate
        cube(motorPlateSize + 
            [2, 2, 0] * plate, 
            center = true);
        
        // the central hole
        cylinder(h = plate + 1, 
            d = motorCenterHoleDiameter, 
            center = true);
        
        // the four screw holes
        for(x = [-1:2:1], y = [-1:2:1])
            translate([x, y, 0] * motorHoleSpacing/2)
            union() {
                
                // the hole
                cylinder(h = 2*plate, 
                    d = motorHoleDiameter, 
                    center = true);
                
                // the countersink
                cylinder(h = plate, 
                    d = 2*motorHoleDiameter);
            }
        
        
    }
}



module motor_mount_box() {
    
    rotate([0, 0, 90])
    difference() {
        
        // outer shape
        cube(motorSize + [2, 2, 2] * plate, 
            center = true);
        
        // inner shape
        cube(motorSize + [0, 3*plate, 0], 
            center = true);
    }
    
    // add the mounting plate
    translate([1, 0, 0] * (motorSize[0]/2 + plate/2))
    rotate([0, 90, 0])
    rotate([0, 0, 90])
    NEMA_plate();
}



// draw one motor with motor mount
module motor(mount, motor) {

    // draw the motor mount
    if (mount > 0)
        color(colorMounts)
        motor_mount_box();
    
    // draw the motor
    if (motor > 0)
        color(colorMotor)
        cube(motorSize, center = true);
}



// a fork for conveying food
module fork() {
   
    // the extender tube
    color(colorTool)
    translate([0, 0, (armLength - dist - 
        linkSideThickness)/2])
    tube(H = armLength - dist - linkSideThickness, 
        OD = 10, ID = 0);
}



// draw a hollow tube 
// (H = height, OD = outside diameter, ID = inside diameter)
module tube(H, OD, ID) {
    
    difference() {
        
        // outer shape
        cylinder(h = H, d = OD, 
            center = true, $fn = $fn*4);
        
        // inner shape
        cylinder(h = H + 2, d = ID, 
            center = true, $fn = $fn*4);
    }
}


