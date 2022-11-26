
console.log("start");

function showData(json) {
    const obj = JSON.parse(json);

    $("#time").text(obj.time);
    //$("#angle").text(parseFloat(obj.angle).format(2));
    $("#pulses").text(obj.pulses);
    var speedShow = parseFloat(obj.speed).toFixed(2);
    $("#speed").text(speedShow);
    var voltageShow = parseFloat(obj.voltage).toFixed(2);
    $("#voltage").text(voltageShow);
    var targetShow = parseFloat(obj.target).toFixed(2);
    $("#target").text(targetShow);
    $("#control").text(obj.target == 0 ? "Torque" : "Velocity");

    var state = "";
    switch (obj.state) {
        case 0:
            state = "Start";
            break;
        case 1:
            state = "Inactive";
            break;
        case 2:
            state = "Spinta";
            break;
        case 3:
            state = "Frenata";
            break;
        case 4:
            state = "Quasi fine corsa";
            break;
        case 5:
            state = "Fine corsa";
            break;
        case 6:
            state = "Inizio ritorno";
            break;
        case 7:
            state = "Ritorno velocità";
            break;
        case 8:
            state = "Ritorno torque";
            break;
    }

    $("#state").text(state);
}

function sendData() {
    var vmax = $("#input-vmax").val();
    var vmin = $("#input-vmin").val();
    //var ramp = $("#input-ramp").val();

    var railLength = $("#input-rail").val();

    var pulseStart = railLength * $("#input-pulseStart").val();
    var pulseStop = railLength * $("#input-pulseStop").val();
    var pulseEnd = railLength * $("#input-pulseEnd").val();
    /* $("#input-pulseStart").val(pulseStart);
     $("#input-pulseStop").val(pulseStop);
     $("#input-pulseEnd").val(pulseEnd);*/
    /*var pulseStart = $("#input-pulseStart").val();
    var pulseStop = $("#input-pulseStop").val();
    var pulseEnd = $("#input-pulseEnd").val();*/
    var tend = $("#input-tend").val();
    var tbrake = $("#input-tbrake").val();
    var timeoutDuration = $("#input-timeoutDuration").val();

    var txt = "Set;" + vmax + ";" + vmin + ";" + 0 + ";" + pulseStart + ";" + pulseStop + ";" + pulseEnd + ";" + tend + ";" + tbrake + ";" + timeoutDuration;

    console.log("sendData: " + txt);

    sendSetPacket(txt);
}

function clearlog(){
    $("#logDiv").text("");
}

function appendToLog(text) {
    $("#logDiv").append(text + "<br>");
}

eel.expose(showData);
eel.expose(appendToLog);

// funzioni collegamento a python

async function stopDrive() {
    await eel.stop_drive()();
}

async function enableDrive() {
    await eel.enable_drive()();
}

async function resetDrive() {
    await eel.reset_drive()();
}

async function sendSetPacket(txt) {
    await eel.send_set_packet(txt)();
}