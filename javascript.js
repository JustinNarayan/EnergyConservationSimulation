/// Declare Input Variables
let blockMass,
   springConstant,
   compressionDistance,
   rampAngle,
   coefficientKineticFriction; // the μk value

/// Declare constants
const dirUp = 90,
   dirDown = 270,
   dirLeft = 180,
   dirRight = 0;
const SECOND_MS = 1000,
   DECIMAL_PLACES = 5,
   PIXELS_PER_METER = 6,
   PIXELS_PER_NEWTON = 0.5;
const gravAcceleration = 9.8;
const rampDistance = 30; // arbitrary ramp distance away from spring equilibrium
const rampLength = 15; // Arbitrary diagonal ramp length (hypotenuse)
const makeRadian = (angle) => (Math.PI * angle) / 180;
const quadraticFormula = (a, b, c) =>
   (-b - Math.sqrt(Math.pow(b, 2) - 4 * a * c)) / (2 * a); // takes the farthest time in the future

/// Initialize Simulation Variables
let time,
   playing = false,
   playingInterval = null,
   playIncrement = 0.01,
   timeStep = 0;

const sectionEndTimes = {
   // new section starts if time >= sectionEndTimes.<the previous one>
   spring: 0,
   surface: 0,
   ramp: 0,
   air: 0,
};

let maximumSystemEnergy, energyLostToFriction;

let springPotentialEnergy,
   currentSpringDisplacement, // as time goes on, displacement reaches 0 and block leaves
   angularFrequency, // the ω value
   springPeriod,
   springTimeToEquilibrium;

const accumulated = {}; // track the net state effects after every section
let blockKineticEnergy,
   blockPotentialEnergy,
   blockPositionX,
   blockPositionY,
   blockNetVelocity,
   blockVelocityX,
   blockVelocityY,
   generalFrictionalAcceleration, // dependent on μk and block weight
   downRampFrictionalAcceleration, // changes based on ramp angle
   downRampGravAcceleration,
   blockAngle;

/// Generate all relevant kinematics equations
const xPositionFromSpring = (elapsedTime, initialCompression, angularFreq) =>
   -1 * initialCompression * Math.cos(angularFreq * elapsedTime);
const xVelocityFromSpring = (elapsedTime, initialCompression, angularFreq) =>
   initialCompression * angularFreq * Math.sin(angularFreq * elapsedTime);

const xPositionFromSurface = (
   sectionStartTime,
   elapsedTime,
   frictionAcceleration,
   initialVelocity
) =>
   initialVelocity * (elapsedTime - sectionStartTime) +
   (frictionAcceleration / 2) * Math.pow(elapsedTime - sectionStartTime, 2);
const xVelocityFromSurface = (
   sectionStartTime,
   elapsedTime,
   frictionAcceleration
) => frictionAcceleration * (elapsedTime - sectionStartTime);

const xPositionFromRamp = (
   sectionStartTime,
   elapsedTime,
   opposingAcceleration,
   initialXVelocity
) =>
   initialXVelocity * (elapsedTime - sectionStartTime) +
   (opposingAcceleration / 2) * Math.pow(elapsedTime - sectionStartTime, 2);
const yPositionFromRamp = (
   sectionStartTime,
   elapsedTime,
   opposingAcceleration,
   initialYVelocity
) =>
   initialYVelocity * (elapsedTime - sectionStartTime) +
   (opposingAcceleration / 2) * Math.pow(elapsedTime - sectionStartTime, 2);
const netVelocityFromRamp = (
   sectionStartTime,
   elapsedTime,
   opposingAcceleration
) => opposingAcceleration * (elapsedTime - sectionStartTime);

const xPositionFromAir = (sectionStartTime, elapsedTime, xVelocity) =>
   xVelocity * (elapsedTime - sectionStartTime);
const yPositionFromAir = (
   sectionStartTime,
   elapsedTime,
   downwardAcceleration,
   initialYVelocity
) =>
   initialYVelocity * (elapsedTime - sectionStartTime) +
   (downwardAcceleration / 2) * Math.pow(elapsedTime - sectionStartTime, 2);
const yVelocityFromAir = (
   sectionStartTime,
   elapsedTime,
   downwardAcceleration
) => downwardAcceleration * (elapsedTime - sectionStartTime);

/// Force
/// ~ class for each force acting on the box through the simulation
class Force {
   constructor(colour, direction = 0) {
      this.colour = colour;
      this.direction = direction; // degrees, as certain forces have constant, non-input based directions
      this.magnitude = 0; // newtons
   }

   setDirection(direction) {
      this.direction = direction;
   }

   setMagnitude(magnitude) {
      this.magnitude = magnitude;
   }
}

let forceGravity = new Force("darkviolet", dirDown),
   forceNormal = new Force("royalblue"),
   forceSpring = new Force("crimson", dirRight),
   forceFriction = new Force("magenta");

/// Canvas variables
let canvas, width, height, ctx;

let X_ZERO, Y_ZERO; // variables to track the origin coordinates using canvas dimensions

const floorColour = "black",
   floorHeight = 80,
   floorDistanceMarkingsSpacing = 20, // in meters
   floorDistanceMarkingsLength = 48, // in pixels
   floorDistanceMarkingsColour = "white",
   floorDistanceMarkingsWidth = 2,
   springBaseWidth = 20,
   rampColour = "gray";

const springWidthMax = 120,
   springHeight = 40,
   ballDiameter = springHeight;

const xFBD = 320,
   yFBD = 76,
   ballDiameterFBD = 24,
   forceArrowRectWidth = 1.5,
   forceArrowTipWidth = 8,
   forceArrowTipHeight = 6;

const xText = 8,
   yText = 46,
   textColour = "black",
   lineSize = "24",
   textFont = "16px Helvetica";

/// Initialize images
const ballImage = new Image(),
   springImage = new Image();
ballImage.src = "images/ball.png";
springImage.src = "images/spring.png"; // 1080px by 288px

/// Listen for initial page load
window.addEventListener(
   "load",
   () => {
      loadCanvas();
      updateInputs();
   },
   false
);

/// updateInputs()
/// ~ read inputs from user, display numerical values, and update simulation
function updateInputs() {
   /// Read Inputs
   blockMass = parseFloat(document.getElementById("block-mass").value);
   springConstant = parseFloat(
      document.getElementById("spring-constant").value
   );
   compressionDistance = parseFloat(
      document.getElementById("compression-distance").value
   );
   rampAngle = parseFloat(document.getElementById("ramp-angle").value);
   coefficientKineticFriction = parseFloat(
      document.getElementById("coefficient-kinetic-friction").value
   );

   /// Write inputs to displays
   document.getElementById("display-block-mass").innerHTML = `${blockMass} kg`;
   document.getElementById(
      "display-spring-constant"
   ).innerHTML = `${springConstant} N/m`;
   document.getElementById(
      "display-compression-distance"
   ).innerHTML = `${compressionDistance} m`;
   document.getElementById(
      "display-ramp-angle"
   ).innerHTML = `${rampAngle} &deg;`;
   document.getElementById(
      "display-coefficient-kinetic-friction"
   ).innerHTML = `${coefficientKineticFriction}`;

   /// Calculate Constants
   calculateConstants();

   /// Reset time
   time = 0;

   /// Compute values and draw frame
   handleFrame();
}

/// handleFrame();
/// ~ compute the values for a frame and draw it
function handleFrame() {
   computeValues();
   drawToCanvas();
}

/// calculateConstants();
/// ~ from certain user inputs, recalculate the relevant constants; avoids unnecessary
/// recalculations
function calculateConstants() {
   /// Compute the relevant spring variables
   computeSpringValues();

   /// Compute constant acceleration
   generalFrictionalAcceleration =
      -1 * coefficientKineticFriction * gravAcceleration;
   downRampFrictionalAcceleration =
      -1 *
      coefficientKineticFriction *
      gravAcceleration *
      Math.cos(makeRadian(rampAngle));
   downRampGravAcceleration =
      -1 * gravAcceleration * Math.sin(makeRadian(rampAngle));
   downRampNetXAcceleration =
      (downRampFrictionalAcceleration + downRampGravAcceleration) *
      Math.cos(makeRadian(rampAngle));
   downRampNetYAcceleration =
      (downRampFrictionalAcceleration + downRampGravAcceleration) *
      Math.sin(makeRadian(rampAngle));

   /// Calculate the block's end-state after each time section (or when changing references) each section is calculated separately so prior sections can be mentioned
   accumulated.fromSpring = {
      positionX: xPositionFromSpring(
         springTimeToEquilibrium,
         compressionDistance,
         angularFrequency
      ),
      velocityX: xVelocityFromSpring(
         springTimeToEquilibrium,
         compressionDistance,
         angularFrequency
      ),
   };

   accumulated.fromSurface = {
      positionX: 0, // Temp value to be adjusted below using velocity value
      velocityX: Math.max(
         0, // in case, friction halts block before ramp
         Math.sqrt(
            Math.pow(accumulated.fromSpring.velocityX, 2) +
               2 * generalFrictionalAcceleration * rampDistance
         ) || 0
      ), // (v-final)^2 = (v-initial)^2 + 2(displacement) => || 0 if NaN
   };
   accumulated.fromSurface.positionX =
      (Math.pow(accumulated.fromSurface.velocityX, 2) -
         Math.pow(accumulated.fromSpring.velocityX, 2)) /
         (2 * generalFrictionalAcceleration) || rampDistance;

   accumulated.onEnteringRamp = {
      velocityX:
         accumulated.fromSurface.velocityX * Math.cos(makeRadian(rampAngle)),
      velocityY:
         accumulated.fromSurface.velocityX * Math.sin(makeRadian(rampAngle)),
   };

   accumulated.fromRamp = {
      positionX: 0,
      positionY: 0,
      velocityX: Math.max(
         0, // in case, friction halts block before ramp
         Math.sqrt(
            Math.pow(accumulated.onEnteringRamp.velocityX, 2) +
               2 *
                  downRampNetXAcceleration *
                  (rampLength * Math.cos(makeRadian(rampAngle)))
         ) || 0
      ),
      velocityY: Math.max(
         0, // in case friction halts block before ramp
         Math.sqrt(
            Math.pow(accumulated.onEnteringRamp.velocityY, 2) +
               2 *
                  downRampNetYAcceleration *
                  (rampLength * Math.sin(makeRadian(rampAngle)))
         ) || 0
      ),
   };
   accumulated.fromRamp.positionX =
      accumulated.fromSurface.positionX +
      (Math.pow(accumulated.fromRamp.velocityX, 2) -
         Math.pow(accumulated.onEnteringRamp.velocityX, 2)) /
         (2 * downRampNetXAcceleration);
   accumulated.fromRamp.positionY =
      (Math.pow(accumulated.fromRamp.velocityY, 2) -
         Math.pow(accumulated.onEnteringRamp.velocityY, 2)) /
      (2 * downRampNetYAcceleration);

   accumulated.onLanding = {
      positionX:
         accumulated.fromRamp.positionX +
         accumulated.fromRamp.velocityX *
            quadraticFormula(
               -gravAcceleration / 2,
               accumulated.fromRamp.velocityY,
               accumulated.fromRamp.positionY
            ),
      positionY: 0,
      velocityX: accumulated.fromRamp.velocityX,
      velocityY: -Math.sqrt(
         Math.pow(accumulated.fromRamp.velocityY, 2) +
            2 * (-gravAcceleration * -accumulated.fromRamp.positionY)
      ),
   };

   /// Compute the sectionEndTimes
   computeSectionEndTimes();
}

/// playPause();
/// ~ play and pause the simulation
function playPause() {
   playing = !playing;
   document.getElementById("play-pause").innerHTML = playing
      ? "Press to Pause"
      : "Press to Play";
   if (playing)
      playingInterval = setInterval(() => {
         time = Math.min(time + playIncrement, getMaxTime());
         handleFrame();
      }, SECOND_MS * playIncrement);
   else clearInterval(playingInterval);
}

/// reset();
/// ~ reset the simulation while maintaing current inputs
function reset() {
   time = 0;
   handleFrame();
}

/// updateTimeStep();
/// ~ retrieve manual time step value
function updateTimeStep() {
   var stepInput = document.getElementById("time-step");
   if (!isNaN(stepInput.value) && stepInput.value > 0) {
      timeStep = Math.abs(parseFloat(stepInput.value));
   } else {
      alert("Please input a valid positive number for the manual time step");
      stepInput.value = timeStep;
   }
}

/// step(direction);
/// direction (number) (-1 || 1): which direction to step through time
/// ~ manually increment/decrement time
function step(direction) {
   if (playing) playPause();

   if (!isNaN(timeStep)) {
      if (timeStep == 0) {
         alert(
            "Your current manual time step is 0 seconds, please input a valid positive number."
         );
         return;
      }
      if (direction < 0) time = Math.max(0, time - timeStep);
      if (direction > 0) time = Math.min(time + timeStep, getMaxTime());
   }

   handleFrame();
}

/// computeValues();
/// ~ using all user inputs, calculate all relevant values for the simulation
function computeValues() {
   /// Compute the position- and velocity- state of the block
   computeBlockValues();

   /// Compute the relevant energies
   computeEnergy();

   /// Compute the current forces on the block
   computeForces();
}

/// computeSpringValues();
/// ~ calculate the spring period, angular frequnecy ω, and duration of contact with block
function computeSpringValues() {
   // Calculate the period of the spring, angular frequency ω, and the duration of time the block is in contact
   angularFrequency = Math.sqrt(springConstant / blockMass);
   springPeriod = (2 * Math.PI) / angularFrequency;
   springTimeToEquilibrium = springPeriod / 4; // how long the block is touching
}

/// computeSectionEndTimes();
/// ~ calculate the end time (i.e. duration) of each section
function computeSectionEndTimes() {
   // Spring always executes 1/4 of a period
   sectionEndTimes.spring = springTimeToEquilibrium;

   // On surface, block may experience friction, no friction, or stop entirely
   if (coefficientKineticFriction == 0) {
      // block maintains velocity from no friction
      sectionEndTimes.surface =
         sectionEndTimes.spring +
         rampDistance / accumulated.fromSurface.velocityX;
   } else {
      // block slows or stops due to friction
      sectionEndTimes.surface =
         sectionEndTimes.spring +
         (accumulated.fromSurface.velocityX -
            accumulated.fromSpring.velocityX) /
            generalFrictionalAcceleration;
   }

   // On ramp, block may stop from friction/PE, stop from PE, exit with KE after friction/PE, or exit with KE after only PE
   sectionEndTimes.ramp =
      sectionEndTimes.surface +
      (accumulated.fromRamp.velocityX - accumulated.onEnteringRamp.velocityX) /
         downRampNetXAcceleration;

   // In air, block experiences only gravitational acceleration, calculate time from x-distanced travelled at constant velocity in air
   sectionEndTimes.air =
      sectionEndTimes.ramp +
      (accumulated.onLanding.positionX - accumulated.fromRamp.positionX) /
         accumulated.onLanding.velocityX;
}

/// getMaxTime();
/// ~ returns the end time of the last valid section the ball goes through
function getMaxTime() {
   return (
      sectionEndTimes.air || sectionEndTimes.ramp || sectionEndTimes.surface
   );
}

/// computeBlockValues();
/// ~ calculate the position and velocity of the block based on its elapsed interactions with the spring, frictional surfaces, ramp, and air
function computeBlockValues() {
   /// Reset block's state
   blockPositionX = 0;
   blockPositionY = 0;
   blockVelocityX = 0;
   blockVelocityY = 0;
   blockAngle = 0;

   // Based on the sections which have fully elapsed, calculate accumulated state values
   if (time >= sectionEndTimes.spring) {
      // After the surface
      blockPositionX = accumulated.fromSpring.positionX;
      blockVelocityX = accumulated.fromSpring.velocityX;
   }
   if (time >= sectionEndTimes.surface) {
      // After the surface
      blockPositionX = accumulated.fromSurface.positionX;
      blockVelocityX = accumulated.onEnteringRamp.velocityX;
      blockVelocityY = accumulated.onEnteringRamp.velocityY;
   }
   if (time >= sectionEndTimes.ramp) {
      blockPositionX = accumulated.fromRamp.positionX;
      blockPositionY = accumulated.fromRamp.positionY;
      blockVelocityX = accumulated.fromRamp.velocityX;
      blockVelocityY = accumulated.fromRamp.velocityY;
   }

   /// Based on current section, calculate in-section state values
   if (time < sectionEndTimes.spring) {
      // Spring
      blockVelocityX += xVelocityFromSpring(
         time,
         compressionDistance,
         angularFrequency
      );
      blockPositionX += xPositionFromSpring(
         time,
         compressionDistance,
         angularFrequency
      );
   } else if (time < sectionEndTimes.surface) {
      // Surface
      blockVelocityX += xVelocityFromSurface(
         sectionEndTimes.spring,
         time,
         generalFrictionalAcceleration
      );
      blockPositionX += xPositionFromSurface(
         sectionEndTimes.spring,
         time,
         generalFrictionalAcceleration,
         accumulated.fromSpring.velocityX
      );
   } else if (time < sectionEndTimes.ramp) {
      // Ramp
      blockAngle = rampAngle;

      blockVelocityX +=
         netVelocityFromRamp(
            sectionEndTimes.surface,
            time,
            downRampFrictionalAcceleration + downRampGravAcceleration
         ) * Math.cos(makeRadian(blockAngle));
      blockVelocityY +=
         netVelocityFromRamp(
            sectionEndTimes.surface,
            time,
            downRampFrictionalAcceleration + downRampGravAcceleration
         ) * Math.sin(makeRadian(blockAngle));

      blockPositionX += xPositionFromRamp(
         sectionEndTimes.surface,
         time,
         downRampNetXAcceleration,
         accumulated.onEnteringRamp.velocityX
      );
      blockPositionY += yPositionFromRamp(
         sectionEndTimes.surface,
         time,
         downRampNetYAcceleration,
         accumulated.onEnteringRamp.velocityY
      );
   } else {
      // In the air until simulation ends
      // Air

      // blockVelocityX constant in section
      blockVelocityY += yVelocityFromAir(
         sectionEndTimes.ramp,
         time,
         -1 * gravAcceleration
      );

      blockPositionX += xPositionFromAir(
         sectionEndTimes.ramp,
         time,
         accumulated.fromRamp.velocityX
      );
      blockPositionY += yPositionFromAir(
         sectionEndTimes.ramp,
         time,
         -1 * gravAcceleration,
         accumulated.fromRamp.velocityY
      );
   }

   // Net Block Velocity
   blockNetVelocity = Math.sqrt(
      Math.pow(blockVelocityX, 2) + Math.pow(blockVelocityY, 2)
   );
}

/// computeEnergy();
/// ~ calculate the kinetic, potential, and lost energy in the simulation
function computeEnergy() {
   /// Maximum System Energy
   maximumSystemEnergy =
      (1 / 2) * springConstant * Math.pow(compressionDistance, 2);

   /// Spring Potential Energy
   currentSpringDisplacement = Math.abs(Math.min(blockPositionX, 0)); // once the block leaves the spring, energy = 0
   springPotentialEnergy =
      (1 / 2) * springConstant * Math.pow(currentSpringDisplacement, 2);

   /// Block Kinetic Energy
   blockKineticEnergy = (1 / 2) * blockMass * Math.pow(blockNetVelocity, 2);

   /// Block Potential Energy
   blockPotentialEnergy = blockMass * gravAcceleration * blockPositionY;

   /// Energy Lost To Friction
   energyLostToFriction =
      maximumSystemEnergy -
      (springPotentialEnergy + blockKineticEnergy + blockPotentialEnergy);
}

/// computeForces();
/// ~ calculate the forces experienced by the block
function computeForces() {
   forceGravity.setMagnitude(blockMass * gravAcceleration);

   forceNormal.setMagnitude(
      time < sectionEndTimes.ramp
         ? forceGravity.magnitude * Math.cos(makeRadian(blockAngle))
         : 0
   );
   forceNormal.setDirection(dirUp + blockAngle);

   forceSpring.setMagnitude(springConstant * currentSpringDisplacement);

   forceFriction.setMagnitude(
      time >= sectionEndTimes.spring &&
         time < sectionEndTimes.ramp /* has friction */
         ? coefficientKineticFriction * forceNormal.magnitude
         : 0
   );
   forceFriction.setDirection(dirLeft + blockAngle);
}

/// loadCanvas()
/// ~ init reference values for canvas
function loadCanvas() {
   canvas = document.getElementById("canvas");
   width = canvas.width;
   height = canvas.height;
   ctx = canvas.getContext("2d");

   //Make images crisper
   ctx.mozImageSmoothingEnabled = false;
   ctx.oImageSmoothingEnabled = false;
   ctx.webkitImageSmoothingEnabled = false;
   ctx.msImageSmoothingEnabled = false;
   ctx.imageSmoothingEnabled = false;

   /// Based on scaling of images/art, calculate the actual x = 0, y = 0
   /// the ball's CENTER-BASE is drawn at (x,y) = (0,0)
   /// though physics y-values increase upward, drawing y-values decrease upward
   X_ZERO = springBaseWidth + springWidthMax;
   Y_ZERO = height - floorHeight;
}

/// drawToCanvas()
/// ~ draw all necessary elements on the canvas
function drawToCanvas() {
   ctx.clearRect(0, 0, width, height);
   drawStaticElements();
   drawDynamicElements();
   drawFreeBodyDiagram();
}

/// drawStaticElements()
/// ~ draw the static elements onto the canvas
function drawStaticElements() {
   ctx.fillStyle = floorColour;
   ctx.beginPath();
   ctx.rect(0, height - floorHeight, width, height); // floor
   ctx.rect(
      0,
      height - (floorHeight + springHeight),
      springBaseWidth,
      springHeight
   ); // spring base
   ctx.fill();
   ctx.fillStyle = rampColour;
   ctx.beginPath();
   let rampBaseX = X_ZERO + PIXELS_PER_METER * rampDistance;
   let rampTopX =
      rampBaseX +
      PIXELS_PER_METER * Math.cos(makeRadian(rampAngle)) * rampLength;
   let rampTopY =
      Y_ZERO - PIXELS_PER_METER * Math.sin(makeRadian(rampAngle)) * rampLength;
   ctx.moveTo(rampBaseX, Y_ZERO);
   ctx.lineTo(rampTopX, rampTopY);
   ctx.lineTo(rampTopX, Y_ZERO);
   ctx.fill();

   // Draw floor distance markings
   for (
      var i = X_ZERO - floorDistanceMarkingsSpacing * PIXELS_PER_METER; // starts at -20
      i < width;
      i += floorDistanceMarkingsSpacing * PIXELS_PER_METER
   ) {
      ctx.fillStyle = floorDistanceMarkingsColour;
      ctx.beginPath();
      ctx.moveTo(i - floorDistanceMarkingsWidth / 2, height - floorHeight);
      ctx.lineTo(
         i - floorDistanceMarkingsWidth / 2,
         height - floorHeight + floorDistanceMarkingsLength
      );
      ctx.lineTo(
         i + floorDistanceMarkingsWidth / 2,
         height - floorHeight + floorDistanceMarkingsLength
      );
      ctx.lineTo(i + floorDistanceMarkingsWidth / 2, height - floorHeight);
      ctx.fill();

      var distance = (i - X_ZERO) / PIXELS_PER_METER;
      var textLeftSpacing = 8;
      ctx.font = textFont;
      ctx.fillText(
         `${distance} m`,
         i + textLeftSpacing,
         height - floorHeight + floorDistanceMarkingsLength
      );
   }
}

/// drawDynamicElements()
/// ~ draw the dynamic elements of the ball and spring
function drawDynamicElements() {
   ctx.beginPath();
   /// The ball's CENTER BASE is drawn at its (x,y)
   ctx.drawImage(
      ballImage,
      X_ZERO + PIXELS_PER_METER * blockPositionX - ballDiameter / 2,
      Y_ZERO - PIXELS_PER_METER * blockPositionY - ballDiameter,
      ballDiameter,
      ballDiameter
   );
   ctx.drawImage(
      springImage,
      springBaseWidth,
      height - (floorHeight + springHeight),
      springWidthMax + PIXELS_PER_METER * Math.min(0, blockPositionX),
      springHeight
   );
}

/// drawFreeBodyDiagram();
/// ~ draw a free body diagram of the ball
function drawFreeBodyDiagram() {
   ctx.drawImage(
      ballImage,
      xFBD - ballDiameterFBD / 2,
      yFBD - ballDiameterFBD / 2,
      ballDiameterFBD,
      ballDiameterFBD
   );
   drawForceArrow(xFBD, yFBD, forceGravity);
   drawForceArrow(xFBD, yFBD, forceNormal);
   drawForceArrow(xFBD, yFBD, forceSpring);
   drawForceArrow(xFBD, yFBD, forceFriction);
   printReadout();
}

/// drawForceArrow();
/// ~ draw a force arrow of certain magnitude and direction for the FBD
function drawForceArrow(xFrom, yFrom, force) {
   if (!force.magnitude) return;

   // Account for all angled forces
   let oppOffset = Math.sin(makeRadian(force.direction));
   let adjOffset = Math.cos(makeRadian(force.direction));

   ctx.fillStyle = force.colour;
   ctx.beginPath();

   ctx.moveTo(
      // Rect Bottom
      xFrom - forceArrowRectWidth * oppOffset,
      yFrom - forceArrowRectWidth * adjOffset
   );
   ctx.lineTo(
      // Rect Top
      xFrom +
         -forceArrowRectWidth * oppOffset +
         force.magnitude * PIXELS_PER_NEWTON * adjOffset,
      yFrom +
         -forceArrowRectWidth * adjOffset -
         force.magnitude * PIXELS_PER_NEWTON * oppOffset
   );
   ctx.lineTo(
      // Arrow Tip Base
      xFrom +
         -forceArrowTipWidth * oppOffset +
         force.magnitude * PIXELS_PER_NEWTON * adjOffset,
      yFrom +
         -forceArrowTipWidth * adjOffset -
         force.magnitude * PIXELS_PER_NEWTON * oppOffset
   );
   ctx.lineTo(
      // Arrow Tip Top
      xFrom +
         forceArrowTipHeight * adjOffset +
         force.magnitude * PIXELS_PER_NEWTON * adjOffset,
      yFrom -
         forceArrowTipHeight * oppOffset -
         force.magnitude * PIXELS_PER_NEWTON * oppOffset
   );
   ctx.lineTo(
      // Arrow Tip Base
      xFrom +
         forceArrowTipWidth * oppOffset +
         force.magnitude * PIXELS_PER_NEWTON * adjOffset,
      yFrom +
         forceArrowTipWidth * adjOffset -
         force.magnitude * PIXELS_PER_NEWTON * oppOffset
   );
   ctx.lineTo(
      // Rect Top
      xFrom +
         forceArrowRectWidth * oppOffset +
         force.magnitude * PIXELS_PER_NEWTON * adjOffset,
      yFrom +
         forceArrowRectWidth * adjOffset -
         force.magnitude * PIXELS_PER_NEWTON * oppOffset
   );
   ctx.lineTo(
      // Rect Bottom
      xFrom + forceArrowRectWidth * oppOffset,
      yFrom + forceArrowRectWidth * adjOffset
   );
   ctx.fill();
}

/// printReadout();
/// ~ give a readout for statistics in the simulation
function printReadout() {
   var numLines = 0;
   ctx.font = textFont;
   // For forces, use each colour
   writeText(
      forceGravity.colour,
      numLines++,
      `Force of Gravity: ${round(forceGravity.magnitude)} N`
   );
   writeText(
      forceNormal.colour,
      numLines++,
      `Normal Force: ${round(forceNormal.magnitude)} N`
   );
   writeText(
      forceSpring.colour,
      numLines++,
      `Spring Force: ${round(forceSpring.magnitude)} N`
   );
   writeText(
      forceFriction.colour,
      numLines++,
      `Force of Friction: ${round(forceFriction.magnitude)} N`
   );

   // For energy/velocity readouts
   var readouts = [
      ``,
      `Time: ${round(time)} seconds ${
         time >= getMaxTime() ? ` » SIMULATION COMPLETE ` : ``
      }`,
      ``,
      `X-Velocity: ${round(blockVelocityX)} meters/second`,
      `Y-Velocity: ${round(blockVelocityY)} meters/second`,
      ``,
      `Maximum System Energy: ${round(Math.abs(maximumSystemEnergy))} Joules`,
      `Spring Potential Energy: ${round(
         Math.abs(springPotentialEnergy)
      )} Joules`,
      `Ball Kinetic Energy: ${round(Math.abs(blockKineticEnergy))} Joules`,
      `Ball Potential Energy: ${round(Math.abs(blockPotentialEnergy))} Joules`,
      `Energy Lost to Friction: ${round(
         Math.abs(energyLostToFriction)
      )} Joules`,
   ];

   readouts.forEach((message) => {
      writeText(textColour, numLines++, message);
   });
}

/// writeText();
/// ~ write text of a certain colour for the readouts
function writeText(colour, numLines, text) {
   ctx.fillStyle = colour;
   ctx.fillText(text, xText, yText + numLines * lineSize);
}

/// round(num);
/// num (number) : the value to round
/// ~ return a number to a fixed number of decimal places
function round(num) {
   return num.toFixed(DECIMAL_PLACES);
}
