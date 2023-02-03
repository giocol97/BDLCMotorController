
console.log("start");

function showData(json) {
    const obj = JSON.parse(json);
    //console.log(json)

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

//sscanf(command.c_str(), "Set;%f;%f;%f;%d;%d;%d;%f;%f;%d;%f;%f;%f;%f", &tmpvmax, &tmpvmin, &tmprampDuration, &tmppulseStart, &tmppulseStop, &tmppulseEnd, &tmptend, &tmptbrake, &tmptimeoutDuration, &tmpvmaxfrenata, &tmpvminfrenata, &tmpcfrenata, &tmpvtocco);

function sendData() {
    var railLength = $("#input-rail").val();

    var vmax = $("#input-vmax").val();
    var vmin = $("#input-vmin").val();
    //var ramp = $("#input-ramp").val();
    var pulseStart = railLength * $("#input-pulseStart").val();
    var pulseStop = railLength * $("#input-pulseStop").val();
    var pulseEnd = railLength * $("#input-pulseEnd").val();
    var tend = $("#input-tend").val();
    var tbrake = $("#input-tbrake").val();
    var timeoutDuration = $("#input-timeoutDuration").val();
    var vmaxfrenata = $("#input-vmaxfrenata").val();
    var vminfrenata = $("#input-vminfrenata").val();
    var cfrenata = $("#input-cfrenata").val();
    var vtocco = $("#input-vtocco").val();

    var txt = "Set;" + vmax + ";" + vmin + ";" + 0 + ";" + pulseStart + ";" + pulseStop + ";" + pulseEnd + ";" + tend + ";" + tbrake + ";" + timeoutDuration + ";" + vmaxfrenata + ";" + vminfrenata + ";" + cfrenata + ";" + vtocco;

    $("#output-box").val(txt);

    console.log("sendData: " + txt);

    sendSetPacket(txt);
}

function setValues() {
    var railLength = $("#input-rail").val();

    //parse txt string and set values to input fields
    var txt = $("#output-box").val();
    var values = txt.split(";");
    $("#input-vmax").val(values[1]);
    $("#input-vmin").val(values[2]);
    $("#input-ramp").val(values[3]);
    $("#input-pulseStart").val((values[4] / railLength).toFixed(2));
    $("#input-pulseStop").val((values[5] / railLength).toFixed(2));
    $("#input-pulseEnd").val((values[6] / railLength).toFixed(2));
    $("#input-tend").val(values[7]);
    $("#input-tbrake").val(values[8]);
    $("#input-timeoutDuration").val(values[9]);
    $("#input-vmaxfrenata").val(values[10]);
    $("#input-vminfrenata").val(values[11]);
    $("#input-cfrenata").val(values[12]);
    $("#input-vtocco").val(values[13]);
}

function clearlog() {
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