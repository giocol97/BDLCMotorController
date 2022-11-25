
console.log("start");

function showData(json){
    const obj = JSON.parse(json);

    $("#time").text(obj.time);
    //$("#angle").text(parseFloat(obj.angle).format(2));
    $("#pulses").text(obj.pulses);
    $("#speed").text(obj.speed);
    $("#voltage").text(obj.voltage);
    $("#target").text(obj.target);
    $("#control").text(obj.target==0?"Torque":"Velocity");

    var state="";
    switch(obj.state){
        case 0:
            state="Start";
            break;
        case 1:
            state="Inactive";
            break;
        case 2:
            state="Spinta";
            break;
        case 3:
            state="Frenata";
            break;
        case 4:
            state="Quasi fine corsa";
            break;
        case 5:
            state="Fine corsa";
            break;
        case 6:
            state="Ritorno velocità";
            break;
        case 7:
            state="Ritorno torque";
            break;
    }

    $("#state").text(state);
}

function sendData(){
    var vmax=$("#input-vmax").val();
    var vmin=$("#input-vmin").val();
    var ramp=$("#input-ramp").val();
    var pulseStart=$("#input-pulseStart").val();
    var pulseStop=$("#input-pulseStop").val();
    var pulseEnd=$("#input-pulseEnd").val();
    var tend=$("#input-tend").val();
    var tbrake=$("#input-tbrake").val();
    var timeoutDuration=$("#input-timeoutDuration").val();

    var txt="Set;"+vmax+";"+vmin+";"+ramp+";"+pulseStart+";"+pulseStop+";"+pulseEnd+";"+tend+";"+tbrake+";"+timeoutDuration;

    sendSetPacket(txt);
}

function appendToLog(text){
    $("#logDiv").append(text + "<br>");
}

eel.expose(showData);
eel.expose(appendToLog);

// funzioni collegamento a python

async function stopDrive(){
    await eel.stop_drive()();
}

async function enableDrive(){
    await eel.enable_drive()();
}

async function resetDrive(){
    await eel.reset_drive()();
}

async function sendSetPacket(txt){
    await eel.send_set_packet(txt)();
}