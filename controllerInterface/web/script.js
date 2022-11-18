
console.log("start");

function showData(json){
    console.log("parsing")
    const obj = JSON.parse(json);

    $("#time").text(obj.time);
    $("#angle").text(obj.angle);
    $("#pulses").text(obj.pulses);
    $("#speed").text(obj.speed);
    $("#voltage").text(obj.voltage);
    $("#target").text(obj.target);
    $("#control").text(obj.target==0?"Torque":"Velocity");

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
