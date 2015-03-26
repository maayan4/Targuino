
//
//   slave repetitive commands
//
////////////////////////////

void slaverepetive(char tempchar)
{
  if ( noevery0dot1sec > millis() )
  {
    noevery0dot1sec = millis();

  }

  if  ( (millis() - noevery0dot1sec) > 100 )
  {
    Serial.println(tempchar); // send command 
    noevery0dot1sec = millis();
  }

}


//
//          buttonprobe
//
///////////////////////////////////////////////

int buttonprobe(int pin)
{

  if (buttonprobedebug)   // DEBUGG!!!
  {

    Serial.println("------- button probe -------"); 
    Serial.println("stats are:"); 

    Serial.print("current time - "); 
    Serial.println(millis());

    Serial.print("button on timer - "); 
    Serial.print(buttonmintimer);
    Serial.print(" - "); 
    Serial.println(millis() - buttonmintimer);

    Serial.print("button off timer - "); 
    Serial.print(buttonofftimer);
    Serial.print(" - "); 
    Serial.println(millis() - buttonmintimer);

    Serial.println();


  }

  if(!digitalRead(pin))
  {

    if (buttonprobedebug)   // DEBUGG!!!
    {

      Serial.println("button is being pressed!!!"); 
      Serial.println();
    }

    if(buttonofftimer) // is the button off time still active?
    {

      if ((millis() - buttonofftimer ) < 250) // check if time is still in effect
      {
        buttonofftimer = millis();
        return 0 ;
      }

      else // if time has passed...
      {
        buttonofftimer = 0; // disable time off timer
      }
    }

    if (buttonmintimer == 0) // button pressed for first time
    {
      buttonmintimer = millis();  // set time on timer
      return 0;
    }

    else // timer was already set up
    {
      if ( ( millis() - buttonmintimer ) > 180 ) // check if the time passed
      {
        buttonmintimer = 0;
        buttonofftimer = millis(); // start counter for button to be off
        return 1;
      }
      else // timer still in effect
      {
        return 0;
      }

    }

  } 
  else
  {
    if (buttonprobedebug)   // DEBUGG!!!
    {

      Serial.println("button is NOT being pressed!!!"); 
      Serial.println();
    }
    buttonmintimer = 0;
    return 0;
  }
}// end of button probe


//
//          magnet probe
//
///////////////////////////////////////////////


int magnetprobe(int pin)
{

  if (magnetprobedebug)   // DEBUGG!!!
  {

    Serial.println("------- inside digital pulse -------"); 
    Serial.println("stats are:"); 

    Serial.print("current time - "); 
    Serial.println(millis());

    Serial.print("button on timer - "); 
    Serial.print(magnetmintimer);
    Serial.print(" - "); 
    Serial.println(millis() - magnetmintimer);

    Serial.print("button off timer - "); 
    Serial.print(magnetofftimer);
    Serial.print(" - "); 
    Serial.println(millis() - magnetmintimer);

    Serial.println();


  }

  if(!digitalRead(pin))
  {

    if (magnetprobedebug)   // DEBUGG!!!
    {

      Serial.println("button is being pressed!!!"); 
      Serial.println();
    }

    if(magnetofftimer) // is the button off time still active?
    {

      if ((millis() - magnetofftimer ) < 1) // check if time is still in effect
      {
        magnetofftimer = millis();
        return 0 ;
      }

      else // if time has passed...
      {
        magnetofftimer = 0; // disable time off timer
      }
    }

    if (magnetmintimer == 0) // button pressed for first time
    {
      magnetmintimer = millis();  // set time on timer
      return 0;
    }

    else // timer was already set up
    {
      if ( ( millis() - magnetmintimer ) > 1 ) // check if the time passed
      {
        magnetmintimer = 0;
        magnetofftimer = millis(); // start counter for button to be off
        return 1;
      }
      else // timer still in effect
      {
        return 0;
      }

    }

  } 

  else
  {
    if (magnetprobedebug)   // DEBUGG!!!
    {

      Serial.println("button is NOT being pressed!!!"); 
      Serial.println();
    }
    magnetmintimer = 0;
    return 0;
  }

} // end of magnetprobe


//
//          pid probe
//
///////////////////////////////////////////////


int PIDprobe(int pin)
{

  if (PIDprobedebug)   // DEBUGG!!!
  {

    Serial.println("------- inside digital pulse -------"); 
    Serial.println("stats are:"); 

    Serial.print("current time - "); 
    Serial.println(millis());

    Serial.print("button on timer - "); 
    Serial.print(PIDmintimer);
    Serial.print(" - "); 
    Serial.println(millis() - PIDmintimer);

    Serial.print("button off timer - "); 
    Serial.print(PIDofftimer);
    Serial.print(" - "); 
    Serial.println(millis() - PIDmintimer);

    Serial.println();


  }

  if(!digitalRead(pin))
  {

    if (PIDprobedebug)   // DEBUGG!!!
    {

      Serial.println("button is being pressed!!!"); 
      Serial.println();
    }

    if(PIDofftimer) // is the button off time still active?
    {

      if ((millis() - PIDofftimer ) < 5) // check if time is still in effect
      {
        PIDofftimer = millis();
        return 0 ;
      }

      else // if time has passed...
      {
        PIDofftimer = 0; // disable time off timer
      }
    }

    if (PIDmintimer == 0) // button pressed for first time
    {
      PIDmintimer = millis();  // set time on timer
      return 0;
    }

    else // timer was already set up
    {
      if ( ( millis() - PIDmintimer ) > 5 ) // check if the time passed
      {
        PIDmintimer = 0;
        PIDofftimer = millis(); // start counter for button to be off
        return 1;
      }
      else // timer still in effect
      {
        return 0;
      }

    }

  } 

  else
  {
    if (PIDprobedebug)   // DEBUGG!!!
    {

      Serial.println("button is NOT being pressed!!!"); 
      Serial.println();
    }
    PIDmintimer = 0;
    return 0;
  }

} // end of PID probe

//
//          pid probe 2
//
///////////////////////////////////////////////


int PIDprobe2(int pin)
{

  if (PIDprobedebug)   // DEBUGG!!!
  {

    Serial.println("------- inside digital pulse -------"); 
    Serial.println("stats are:"); 

    Serial.print("current time - "); 
    Serial.println(millis());

    Serial.print("button on timer - "); 
    Serial.print(PIDmintimer);
    Serial.print(" - "); 
    Serial.println(millis() - PIDmintimer);

    Serial.print("button off timer - "); 
    Serial.print(PIDofftimer);
    Serial.print(" - "); 
    Serial.println(millis() - PIDmintimer);

    Serial.println();


  }

  if(!digitalRead(pin))
  {

    if (PIDprobedebug)   // DEBUGG!!!
    {

      Serial.println("button is being pressed!!!"); 
      Serial.println();
    }

    if(PIDofftimer) // is the button off time still active?             //////////////   FLAG???!
    {

        return 0 ;
      
    }
    
    PIDofftimer = 1;  // flagg

    return 1;
  } 

  else
  {
   PIDofftimer = 0;   // flagg
    return 0;
  }

} // end of PID probe 2


//
//        Calibration
//
/////////////////////////////////////////

unsigned int calibration() // 
{  
  if (callibrationdebug)
  {
    Serial.println("------- inside callibration -------"); 
    Serial.println();
  }

  unsigned long calibrationslavetime = 0;
  int counter = 0;    //stores how many clics
  unsigned long pressingprotectiontimer = 0;

  // stop all motors
  ///////////////////

  if (callibrationdebug)
  {
    Serial.println("all motors stopped"); 
    Serial.println();
  }

  analogWrite(MotorPWMPin, 0);  // stop main motor
  Serial.println('N');  // stop slave motor
  ///////////////////////////////////////////////////////


  // start slave motor
  ///////////////////

  if (callibrationdebug)
  {
    Serial.println("start slave motor on full power"); 
    Serial.print("start master motor with callibration brake speed - "); 
    Serial.println(calibrationbreakspeed); 
    Serial.println();
  }


  //////////////////////////////////////////////////////////



  //
  //   first callibration run
  //
  //   running slave motor 
  //   waiting for button or 15 seconds to pass
  //
  /////////////////////////////////////

  if (callibrationdebug)
  {
    Serial.println("first callibration run!!"); 
    Serial.println("running slave motor"); 
    Serial.println("waiting for 15 seconds or for the button to be pressed "); 
    Serial.println();

  }

  Serial.println('Y');  // start slave motor

  analogWrite(MotorPWMPin, calibrationbreakspeed);  // master motor callibration brake


  calibrationslavetime = millis();

  while ( ( !( buttonprobe(button) ) )&&( ( millis() - calibrationslavetime ) < 15000  )  )  // waiting while button not pressed
  {
  }

  if (callibrationdebug)
  {
    Serial.println("first callibration run is done!!!!"); 
    Serial.print("time passed "); 
    Serial.println( (millis() - calibrationslavetime)/1000 ); 
    Serial.println();
  }


  Serial.println('N'); // stop slave motor

  delay (150);


  if (callibrationdebug)
  {
    Serial.println("second callibration run!!"); 
    Serial.println("running master motor and counting pulses"); 
    Serial.println("waiting for 15 seconds or for the button to be pressed "); 
    Serial.println();

  }

  analogWrite(MotorPWMPin, mastermotorspeed);  // start master motor
  Serial.println('b'); // slave calibration break speed


  pressingprotectiontimer = millis(); // button protection timer started

  if (callibrationdebug)
  {
    Serial.println("starting 3 seconds button protection timer "); 
    Serial.println();
  }


  //
  //   second callibration run
  //
  //   running master motor 
  //   counting pulses 
  //   waiting for button or 15 seconds to pass
  //
  /////////////////////////////////////

  calibrationslavetime = millis();

  while ( ( millis() - pressingprotectiontimer ) < 3000 )  // waiting while button not pressed
  {
    counter = counter + magnetprobe(magnet);
    // slaverepetive('b');  // send repetitive 
  }

  if (callibrationdebug)
  {
    Serial.println(" 3 seconds button protection done "); 
    Serial.print("pulses = "); 
    Serial.println( counter ); 
    Serial.println();
  }


  if (callibrationdebug)
  {
    Serial.println(" continuing second callibration tun with no button protection "); 
  }

  calibrationslavetime = millis();

  while ( ( !( buttonprobe(button) ) )&&( ( millis() - calibrationslavetime ) < 15000  )  )  // waiting while button not pressed
  {
    counter = counter + magnetprobe(magnet);
    //  slaverepetive('b');
  }

  if (callibrationdebug)
  {
    Serial.println("second callibration run is done!!!!"); 
    Serial.print("time passed "); 
    Serial.println( (millis() - 3000 - calibrationslavetime)/1000 ); 
    Serial.println();
    Serial.print("total pulses = "); 
    Serial.println( counter ); 
    Serial.println();
  }


  analogWrite(MotorPWMPin, 0);  // stop main motor
  Serial.println('N');  // stop slave motor

  return counter;

}
