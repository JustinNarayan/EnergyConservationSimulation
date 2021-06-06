/// Declare Input Variables
let blockMass,
   springConstant,
   compressionDistance,
   rampAngle,
   coefficientKineticFriction; // the μk value

/// Declare constants
const gravAcceleration = 9.8;
const rampDistance = 10; // arbitrary ramp distance away from spring equilibrium
const rampLength = 10; // Arbitrary diagonal ramp length (hypotenuse)
const makeRadian = (angle) => (Math.PI * angle) / 180;
const floatErrorMargin = Math.pow(10, -8);

/// Initialize Simulation Variables
let time;
const sectionEndTimes = {
   // new section starts if time >= sectionEndTimes.<the previous one>
   spring: 0,
   surface: 0,
   ramp: 0,
   air: 0,
};

let maximumSystemEnergy, energyLostToFriction;

let forceGravity, forceNormal, forceSpring, forceFriction;

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
   frictionalAcceleration, // changes based on ramp angle
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
const netVelocityFromRamp = (
   sectionStartTime,
   elapsedTime,
   opposingAcceleration
) => opposingAcceleration * (elapsedTime - sectionStartTime);

/// Listen for initial page load
window.addEventListener("load", updateInputs, false);

/// updateInputs(recomputeConstants)
/// recomputeConstants (boolean) : if the simulation's constants must be recomputed
/// ~ read inputs from user, display numerical values, and update simulation
function updateInputs(recomputeConstants) {
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

   time = parseFloat(document.getElementById("time").value);
   document.getElementById("display-time").innerHTML = `${time}`;

   /// Calculate Constants
   if (recomputeConstants) {
      calculateConstants();
   }

   /// Update temporary data readout
   computeValues();
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
   downRampGravAcceleration =
      -1 * gravAcceleration * Math.sin(makeRadian(rampAngle));

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
      positionX: rampDistance,
      velocityX: Math.max(
         0, // in case, friction halts block before ramp
         Math.sqrt(
            Math.pow(accumulated.fromSpring.velocityX, 2) +
               2 * generalFrictionalAcceleration * rampDistance
         ) || 0
      ), // (v-final)^2 = (v-initial)^2 + 2(displacement) => || 0 if NaN
   };
   accumulated.onEnteringRamp = {
      velocityX:
         accumulated.fromSurface.velocityX * Math.cos(makeRadian(rampAngle)),
      velocityY:
         accumulated.fromSurface.velocityX * Math.sin(makeRadian(rampAngle)),
   };
   /// Compute the sectionEndTimes
   computeSectionEndTimes();

   console.log(sectionEndTimes);
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

   updateTempDataOutput();
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
   if (
      Math.abs(
         accumulated.fromSurface.velocityX - accumulated.fromSpring.velocityX
      ) < floatErrorMargin
   ) {
      // no friction
      sectionEndTimes.surface =
         sectionEndTimes.spring +
         rampDistance / accumulated.fromSurface.velocityX;
   } else if (accumulated.fromSurface.velocityX <= 0) {
      // block stops
      sectionEndTimes.surface = Number.MAX_SAFE_INTEGER;
   } else {
      // block slows a bit => (velocity-change) / (acceleration) = time
      sectionEndTimes.surface =
         sectionEndTimes.spring +
         (accumulated.fromSurface.velocityX -
            accumulated.fromSpring.velocityX) /
            generalFrictionalAcceleration;
   }
   sectionEndTimes.ramp = 1500; //TEMP
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

   // Calculate frictional acceleration opposite direction of motion - changes based on current section
   frictionalAcceleration =
      generalFrictionalAcceleration * Math.cos(makeRadian(blockAngle));

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

   /// Based on current section, calculate in-section state values
   if (time < sectionEndTimes.spring) {
      // Spring
      blockPositionX += xPositionFromSpring(
         time,
         compressionDistance,
         angularFrequency
      );
      blockVelocityX += xVelocityFromSpring(
         time,
         compressionDistance,
         angularFrequency
      );
   } else if (time < sectionEndTimes.surface) {
      // Surface
      blockPositionX += xPositionFromSurface(
         sectionEndTimes.spring,
         time,
         frictionalAcceleration,
         accumulated.fromSpring.velocityX
      );
      blockVelocityX += xVelocityFromSurface(
         sectionEndTimes.spring,
         time,
         frictionalAcceleration
      );
   } else if (time < sectionEndTimes.ramp) {
      //Ramp
      blockAngle = rampAngle;

      blockVelocityX +=
         netVelocityFromRamp(
            sectionEndTimes.surface,
            time,
            frictionalAcceleration + downRampGravAcceleration
         ) * Math.cos(makeRadian(blockAngle));
      blockVelocityY +=
         netVelocityFromRamp(
            sectionEndTimes.surface,
            time,
            frictionalAcceleration + downRampGravAcceleration
         ) * Math.sin(makeRadian(blockAngle));
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
   forceGravity = blockMass * gravAcceleration;
   forceNormal = forceGravity * Math.cos(makeRadian(blockAngle));
   forceSpring = springConstant * currentSpringDisplacement;
   forceFriction =
      time >= sectionEndTimes.spring &&
      time < sectionEndTimes.ramp /* has friction */
         ? coefficientKineticFriction * forceNormal
         : 0;
}

/// updateTempDataOutput();
/// ~ calculate some data to check the simulation's physics engine
function updateTempDataOutput() {
   let page = document.getElementById("temp-data-output");

   /// Clear page then write information
   page.innerHTML = ``;
   page.innerHTML += `Maximum Energy: ${maximumSystemEnergy} <br>`;
   page.innerHTML += `Spring PE: ${springPotentialEnergy} <br>`;
   page.innerHTML += `Block KE: ${blockKineticEnergy} <br>`;
   page.innerHTML += `Block PE: ${blockPotentialEnergy} <br>`;
   page.innerHTML += `Energy Lost: ${energyLostToFriction} <br>`;
   page.innerHTML += `<br>`;
   page.innerHTML += `F-grav: ${forceGravity} <br>`;
   page.innerHTML += `F-normal: ${forceNormal} <br>`;
   page.innerHTML += `F-spring: ${forceSpring} <br>`;
   page.innerHTML += `F-friction: ${forceFriction} <br>`;
   page.innerHTML += `<br>`;
   page.innerHTML += `Block X: ${blockPositionX} <br>`;
   page.innerHTML += `Block Y: ${blockPositionY} <br>`;
   page.innerHTML += `Block Velocity X: ${blockVelocityX} <br>`;
   page.innerHTML += `Block Velocity Y: ${blockVelocityY} <br>`;
   page.innerHTML += `Block Velocity Net: ${blockNetVelocity} <br>`;
}
