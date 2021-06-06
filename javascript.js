/// Declare Input Variables
let blockMass,
   springConstant,
   compressionDistance,
   rampAngle,
   coefficientKineticFriction; // the μk value

/// Declare constants
const gravAcceleration = 9.8;
const rampDistance = 10; // arbitrary ramp distance away from spring equilibrium

/// Initialize Simulation Variables
let time,
   sectionEndTimes = {
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

const accumulated = {}; // track the state effects from (and ONLY from) each individual section
let blockKineticEnergy,
   blockPotentialEnergy,
   blockPositionX,
   blockPositionY,
   blockVelocity,
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

   /// Compute general frictional acceleration
   generalFrictionalAcceleration =
      -1 * coefficientKineticFriction * gravAcceleration;

   /// Calculate the block's end-state after each time section
   /// each section is calculated separately so prior sections can be referenced
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
      velocityX:
         Math.max(
            0, // in case, friction halts block before ramp
            Math.sqrt(
               Math.pow(accumulated.fromSpring.velocityX, 2) +
                  2 * generalFrictionalAcceleration * rampDistance
            ) || 0
         ) - accumulated.fromSpring.velocityX, // (v-final)^2 = (v-initial)^2 + 2(displacement) => || 0 if NaN
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
   sectionEndTimes.spring = springTimeToEquilibrium;
   sectionEndTimes.surface =
      sectionEndTimes.spring +
      accumulated.fromSurface.velocityX / generalFrictionalAcceleration; // (velocity-change) / (acceleration) = time
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

   // Calculate frictional acceleration opposite direction of motion - can't use force data as it would be from prior frame
   frictionalAcceleration =
      generalFrictionalAcceleration * Math.cos(blockAngle);

   /// Based on current section, calculate in-section state values
   if (time < sectionEndTimes.spring) {
      // Spring
      blockPositionX = xPositionFromSpring(
         time,
         compressionDistance,
         angularFrequency
      );
      blockVelocityX = xVelocityFromSpring(
         time,
         compressionDistance,
         angularFrequency
      );
   } else if (time < sectionEndTimes.surface) {
      // Surface
      blockPositionX = xPositionFromSurface(
         sectionEndTimes.spring,
         time,
         frictionalAcceleration,
         accumulated.fromSpring.velocityX
      );
      blockVelocityX = xVelocityFromSurface(
         sectionEndTimes.spring,
         time,
         frictionalAcceleration
      );
   }

   // Based on the sections which have fully elapsed, calculate accumulated state values
   if (time >= sectionEndTimes.spring) {
      // After the surface
      blockPositionX += accumulated.fromSpring.positionX;
      blockVelocityX += accumulated.fromSpring.velocityX;
   }
   if (time >= sectionEndTimes.surface) {
      // After the surface
      blockPositionX += accumulated.fromSurface.positionX;
      blockVelocityX += accumulated.fromSurface.velocityX;
   }

   // Net Block State
   blockVelocity = Math.sqrt(
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
   blockKineticEnergy = (1 / 2) * blockMass * Math.pow(blockVelocity, 2);

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
   forceNormal = forceGravity * Math.cos(blockAngle);
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
   page.innerHTML += `Spring End Time: ${sectionEndTimes.spring} <br>`;
   page.innerHTML += `Block X: ${blockPositionX} <br>`;
   page.innerHTML += `Block Velocity X: ${blockVelocityX} <br>`;
}
