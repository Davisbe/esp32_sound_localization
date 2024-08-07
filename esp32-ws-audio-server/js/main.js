/* basically everything is hardcoded for 16 bit depth
    const AUDIO_IN_BITDEPTH = 16; */
// Sample rate of the audio input:
const AUDIO_IN_SAMPLESIZE = 44100;
// Buffer that gets passed to the WASM module:
const AUDIO_WASM_BUFFER_SIZE = 512 * 100;
// connect button, sets up WS and other important stuff
var connectBtn = document.getElementById("connectBtn");
connectBtn.disabled = false; // enable the button by default

var recordAllBtn = document.getElementById("recordAllBtn");
recordAllBtn.disabled = true;

var checkSyncBtn = document.getElementById("checkSyncBtn");
checkSyncBtn.disabled = true;

var startLocalizationBtn = document.getElementById("startLocalizationBtn");
startLocalizationBtn.disabled = true;
// place to display average time difference between the ESPs:
var times_diff = document.getElementById("times_diff");
// map for the ESP unique IDs and indexes (0, 1, 2):
var nic_addresses = new Map();
// store last times that audio was received from the ESPs:
var last_times = new BigInt64Array([0n, 0n, 0n]);
// stores most info and UI elements for the ESPs:
var waveforms = [];
var isLocalizationRunning = false;
var isLocalizationRecording = false;
var localizationResult = [];
// create objects for each ESP in waveforms[]
for (var i = 0; i < 3; i++) {
    player = new PCMPlayer({
        inputCodec: 'Int16',
        channels: 1,
        sampleRate: AUDIO_IN_SAMPLESIZE,
    });

    let canvasCtx = document.getElementById('graph'.concat(i + 1)).getContext('2d');

    waveforms[i] = {
        coordinates: [0, 0],
        nic: 0,
        canvasCtx: canvasCtx,
        dataArray: new Array(128).fill(canvasCtx.canvas.height * 1.5),
        audioBuffer: {
            time: 0n,
            data: undefined,
            freed_chunks: 0,   // so we dont store large audio data in memory
            file_view: undefined,
            audio_blob: undefined,
            audio_url: "",
        },
        worker: new Worker('js/worker.js'),
        player: player,
        pauseBtn: document.getElementById("pauseBtn".concat(i + 1)),
        continueBtn: document.getElementById("continueBtn".concat(i + 1)),
        ESPlocalTimeDiff: document.getElementById("times_time".concat(i + 1)),
        recorderBtn: document.getElementById("recordBtn".concat(i + 1)),
        recordingTimeDur: document.getElementById("recordingTimeDur".concat(i + 1)),
        isSyncSampleTaken: false,
    };

    waveforms[i].pauseBtn.disabled = true;
    waveforms[i].continueBtn.disabled = true;
    waveforms[i].recorderBtn.disabled = true;
}

var micDistance1Input = document.getElementById("micDistance1");
var micDistance2Input = document.getElementById("micDistance2");
var micDistance3Input = document.getElementById("micDistance3");
var micGraphScale = document.getElementById("micGraphScale");

micDistance1Input.addEventListener('change', function () {
    updateMicLocations();
});
micDistance2Input.addEventListener('change', function () {
    updateMicLocations();
});
micDistance3Input.addEventListener('change', function () {
    updateMicLocations();
});
micGraphScale.addEventListener('change', function () {
    updateMicGraph();
});

function addDarkmodeWidget() {
    new Darkmode().showWidget();
}
window.addEventListener('load', addDarkmodeWidget);


function locationTest() {
    let temperature = 24;
    let c = 331 + (0.61 * temperature);
    let toas = new BigInt64Array([13107n, 0n, 34641n]);
    let mloc = [
        new Float64Array([0.0, 0.0]),
        new Float64Array([1.5, 0.0]),
        new Float64Array([0.75, 1.3])
    ];

    // Allocate memory for the result_x and result_y variables
    let result_xPtr = Module._malloc(8); // double is 8 bytes
    let result_yPtr = Module._malloc(8);

    // Allocate memory for the 'toas' array
    let toasPtr = Module._malloc(toas.byteLength);
    Module.HEAP64.set(toas, toasPtr / toas.BYTES_PER_ELEMENT);

    // Allocate memory for the 'mloc' array
    let mlocPtr = Module._malloc(mloc.length * mloc[0].byteLength);
    mloc.forEach((arr, index) => {
        Module.HEAPF64.set(arr, (mlocPtr / arr.BYTES_PER_ELEMENT) + (index * arr.length));
    });

    // Call the 'getLocation' function with pointers
    Module._getLocation(temperature, toasPtr, mlocPtr, result_xPtr, result_yPtr);

    let result_x = Module.HEAPF64[result_xPtr / 8];
    let result_y = Module.HEAPF64[result_yPtr / 8];
    console.log('Result X:', result_x, 'Result Y:', result_y);

    // Free the allocated memory
    Module._free(toasPtr);
    Module._free(mlocPtr);
    Module._free(result_xPtr);
    Module._free(result_yPtr);
}

function absBigInt(n) {
    return (n < 0n) ? -n : n;
}

async function calcTimeDifference() {
    // Calculate the average difference between the last_times:
    /*
        ESP32S3s send 441000 samples per second
        They send 512 samples per packet
        So new packets get sent every ~11,6ms (512/44100 * 1000ms = ~11,6ms)

        Worst case scenario (roughly), the difference error is
        (11,6 + 11,6 + 0) / 3 = 7,733ms
    */
    var diff1 = Number(absBigInt(last_times[0] - last_times[1]));
    var diff2 = Number(absBigInt(last_times[0] - last_times[2]));
    var diff3 = Number(absBigInt(last_times[1] - last_times[2]));
    var average_diff = (diff1 + diff2 + diff3) / 3;
    var average_diff_milliseconds = average_diff / 1000;


    times_diff.innerHTML = Math.round(average_diff_milliseconds);

    setTimeout(calcTimeDifference, 300);
}

waveforms.forEach(function (wf, idx) {
    wf.worker.addEventListener('message', function (e) {

        // Add new data to the data array, which will be used
        // to draw the waveform
        wf.dataArray = wf.dataArray.concat(e.data + wf.canvasCtx.canvas.height)
        wf.dataArray.splice(0, 1)

        // Draw waveform
        wf.canvasCtx.clearRect(0, 0, wf.canvasCtx.canvas.width, wf.canvasCtx.canvas.height); // Clear the canvas
        wf.canvasCtx.beginPath();
        wf.canvasCtx.lineWidth = 0.5;
        wf.canvasCtx.strokeStyle = "rgb(0 0 0)";
        var sliceWidth = wf.canvasCtx.canvas.width / 128;
        var x = 0;
        for (var i = 0; i < 128; i++) {
            var v = wf.dataArray[i] / 128.0;
            var y = v * wf.canvasCtx.canvas.height / 3;
            if (i === 0) {
                wf.canvasCtx.moveTo(x, y);
            } else {
                wf.canvasCtx.lineTo(x, y);
            }
            x += sliceWidth;
        }
        wf.canvasCtx.lineTo(wf.canvasCtx.canvas.width, wf.canvasCtx.canvas.height / 2);
        wf.canvasCtx.stroke();

    }, false);
});

function writeUTFBytes(view, offset, string) {
    var len = string.length;
    for (var i = 0; i < len; i++) {
        view.setUint8(offset + i, string.charCodeAt(i));
    }
}

function initWAVDataViewHeader(chunk_size, samples_to_record) {
    let buffer = new ArrayBuffer(44 + chunk_size * 2); // 44 bytes for header
    let view = new DataView(buffer);
    // RIFF chunk descriptor
    writeUTFBytes(view, 0, 'RIFF');
    view.setUint32(4, 44 + samples_to_record * 2, true);
    writeUTFBytes(view, 8, 'WAVE');
    // FMT sub-chunk
    writeUTFBytes(view, 12, 'fmt ');
    view.setUint32(16, 16, true);
    view.setUint16(20, 1, true);
    view.setUint16(22, 1, true);
    view.setUint32(24, AUDIO_IN_SAMPLESIZE, true);
    view.setUint32(28, AUDIO_IN_SAMPLESIZE * 2, true);
    view.setUint16(32, 2, true);
    view.setUint16(34, 16, true);
    writeUTFBytes(view, 36, 'data');
    view.setUint32(40, samples_to_record * 2, true);

    return view;
}

function initWAVDataView(size) {
    let buffer = new ArrayBuffer(size * 2);
    let view = new DataView(buffer);

    return view;
}

function feedAudioToRecorder(nicIndex, int16Audio, read_time, record_seconds) {
    var aB = waveforms[nicIndex].audioBuffer;
    var samples_to_record = record_seconds * AUDIO_IN_SAMPLESIZE;
    var chunk_size = 512 * 100;   // to free aB.file_view and aB.data

    if (samples_to_record - (chunk_size * aB.freed_chunks) < chunk_size) {
        chunk_size = samples_to_record - (chunk_size * aB.freed_chunks);
    }

    if (!aB.file_view) {

        if (aB.freed_chunks > 0) {
            aB.file_view = initWAVDataView(chunk_size);
        } else {
            aB.file_view = initWAVDataViewHeader(chunk_size, samples_to_record);
            aB.time = read_time;
            // remove previous blob if it exists
            if (aB.audio_blob) {
                URL.revokeObjectURL(aB.audio_blob);
                aB.audio_blob = undefined;
            }
        }
    }

    aB.data = aB.data ? new Int16Array([].concat(Array.from(aB.data), Array.from(int16Audio))) : new Int16Array(int16Audio);

    if (aB.data.length >= chunk_size) {
        var offset = 0;
        if (aB.freed_chunks == 0) {
            offset = 44;    // for WAV header
        }
        for (var i = 0; i < chunk_size; i++, offset += 2) {
            var s = aB.data[i] / 32768;
            // var s = aB.data[i];
            s = Math.max(-1, Math.min(1, s));
            aB.file_view.setInt16(offset, s < 0 ? s * 0x8000 : s * 0x7FFF, true);
            // aB.file_view.setInt16(offset, s, true);
        }

        if (aB.audio_blob) {
            aB.audio_blob = new Blob([aB.audio_blob, aB.file_view], { type: "audio/wav" });
        } else {
            aB.audio_blob = new Blob([aB.file_view], { type: "audio/wav" });
        }
        aB.freed_chunks++;
        aB.file_view = undefined;
        aB.data = undefined;
    }
    if (aB.audio_blob && ((aB.audio_blob.size - 44) / 2 >= samples_to_record)) {
        return 1;
    }
    return 0;
}


// remember to set aB.data to undefined after done with stuff
function feedAudioToDataBuffer(nicIndex, int16Audio, read_time, record_time) {
    var aB = waveforms[nicIndex].audioBuffer;
    var samples_to_record = Math.round(record_time * AUDIO_IN_SAMPLESIZE);

    if (!aB.data) {
        aB.time = read_time;
    }

    aB.data = aB.data ? new Int16Array([].concat(Array.from(aB.data), Array.from(int16Audio))) : new Int16Array(int16Audio);

    if (aB.data.length >= samples_to_record) {
        return 1;
    }

    return 0;
}

function stopAndDownloadRecording(nicIndex) {
    var aB = waveforms[nicIndex].audioBuffer;
    waveforms[nicIndex].recorderBtn.disabled = false;
    aB.audio_url = URL.createObjectURL(aB.audio_blob);
    console.log("Audio file blob URL created: " + aB.audio_url);
    var a = document.createElement('a');
    document.body.appendChild(a);
    a.style.display = 'none';
    a.href = aB.audio_url;
    a.download = waveforms[nicIndex].nic + '-' + aB.time + '.wav';
    a.click();
    a.remove();
    aB.audio_blob = undefined;
    aB.file_view = undefined;
    aB.data = undefined;
    aB.freed_chunks = 0;
}

function checkSync() {
    console.log('Checking sync...');

    var start_times = [Module._malloc(8), Module._malloc(8), Module._malloc(8)];
    for (var i = 0; i < 3; i++) {
        // Assuming waveforms[i].audioBuffer.time is a BigInt representing an int64_t value
        Module.HEAP64[start_times[i] / 8] = waveforms[i].audioBuffer.time;
    }

    // Allocate memory for the array of pointers (which are int32 in Emscripten's memory)
    let start_timesPtr = Module._malloc(start_times.length * 4); // 4 bytes per pointer

    // Set the pointers in the allocated memory
    for (var i = 0; i < start_times.length; i++) {
        Module.HEAP32[start_timesPtr / 4 + i] = start_times[i];
    }

    // Allocate memory for the results
    var result_mic1Ptr = Module._malloc(8); // int64_t is 8 bytes
    var result_mic2Ptr = Module._malloc(8);
    var result_mic3Ptr = Module._malloc(8);

    var tracksPointers = new Uint32Array(3);
    // Allocate memory for the array of pointers (which are int32 in Emscripten's memory)
    let tracksPointersPtr = Module._malloc(tracksPointers.length * 4); // 4 bytes per int32 pointer

    // Allocate memory and get pointers
    for (let i = 0; i < 3; i++) {
        const int16Array = waveforms[i].audioBuffer.data;
        const bytesPerElement = int16Array.BYTES_PER_ELEMENT;
        const numBytes = int16Array.length * bytesPerElement;
        const ptr = Module._malloc(numBytes); // Allocate memory in the WebAssembly module
        tracksPointers[i] = ptr;
        Module.HEAP16.set(int16Array, ptr / 2); // Copy data into allocated memory
        Module.HEAPU32[tracksPointersPtr / 4 + i] = ptr;
    }

    // Call the 'getLocation' function with pointers
    Module._getSyncTime(start_timesPtr, tracksPointersPtr, BigInt(waveforms[0].audioBuffer.data.length), result_mic1Ptr, result_mic2Ptr, result_mic3Ptr);

    let result_mic1 = Module.HEAP64[result_mic1Ptr / 8];
    let result_mic2 = Module.HEAP64[result_mic2Ptr / 8];
    let result_mic3 = Module.HEAP64[result_mic3Ptr / 8];
    console.log(
        'Result Mic1 (' + waveforms[0].nic + '):', Number(result_mic1) / 1000000,
        'Result Mic2 (' + waveforms[1].nic + '):', Number(result_mic2) / 1000000,
        'Result Mic3 (' + waveforms[2].nic + '):', Number(result_mic3) / 1000000);

    // Free the allocated memory
    for (let ptr of tracksPointers) {
        Module._free(ptr);
    }
    for (let ptr of start_times) {
        Module._free(ptr);
    }
    Module._free(result_mic1Ptr);
    Module._free(result_mic2Ptr);
    Module._free(result_mic3Ptr);

    for (var i = 0; i < 3; i++) {
        var aB = waveforms[i].audioBuffer;
        aB.data = undefined;
        waveforms[i].isSyncSampleTaken = false;
    }

}

function setUpDistanceUI() {
    var micDistance1HTML = document.getElementById("micDistance1name");
    var micDistance2HTML = document.getElementById("micDistance2name");
    var micDistance3HTML = document.getElementById("micDistance3name");

    micDistance1HTML.innerHTML = '#1(' + waveforms[0].nic + ") <--> " + '#2(' + waveforms[1].nic + ')';
    micDistance2HTML.innerHTML = '#1(' + waveforms[0].nic + ") <--> " + '#3(' + waveforms[2].nic + ')';
    micDistance3HTML.innerHTML = '#2(' + waveforms[1].nic + ") <--> " + '#3(' + waveforms[2].nic + ')';
}
updateMicLocations()

function setUpDistanceGraph() {
    var mic_graph = document.getElementById('mic_graph');
    var h = mic_graph.height;
    var w = mic_graph.width;
    var ctx = mic_graph.getContext('2d');

    // draw a border around the canvas (600x600px)
    ctx.strokeRect(0, 0, 600, 600);

    ctx.fillRect(w / 2, h / 2, 4, 4);
    ctx.font = "10px Arial";
    ctx.fillText("[ESP#1]", 308, 300);
    ctx.fillRect(w / 2, h / 4, 4, 4);

}

function updateMicGraph() {
    var scale = Number(micGraphScale.value);
    var mic_graph = document.getElementById('mic_graph');
    var h = mic_graph.height;
    var w = mic_graph.width;
    var ctx = mic_graph.getContext('2d');
    ctx.clearRect(0, 0, w, h);
    ctx.font = "10px Arial";

    xy1 = waveforms[0].coordinates;
    xy2 = waveforms[1].coordinates;
    xy3 = waveforms[2].coordinates;

    // draw a border around the canvas (600x600 px)
    ctx.strokeRect(0, 0, 600, 600);

    xOffset = xy2[0] / 2 * scale;
    yOffset = xy3[1] / 2 * scale;

    if (localizationResult.length > 0) {
        xr1 = localizationResult[0] * scale + w / 2 - xOffset;
        yr1 = -localizationResult[1] * scale + h / 2 + yOffset;

        ctx.fillStyle = "green";
        ctx.fillRect(xr1, yr1, 4, 4);
        ctx.fillStyle = "#000000";
        ctx.fillText('(' + Math.round(localizationResult[0] * 10) / 10 + ', ' + Math.round(localizationResult[1] * 10) / 10 + ')', xr1, yr1 - 8);

    }

    x1 = (xy1[0] * scale + w / 2) - xOffset;
    y1 = (xy1[1] * scale + h / 2) + yOffset;
    x2 = (xy2[0] * scale + w / 2) - xOffset;
    y2 = (xy2[1] * scale + h / 2) + yOffset;
    x3 = (xy3[0] * scale + w / 2) - xOffset;
    y3 = (-xy3[1] * scale + h / 2) + yOffset;

    ctx.fillRect(x1, y1, 4, 4);
    ctx.fillText(waveforms[0].nic, x1 - 44, y1);

    ctx.fillRect(x2, y2, 4, 4);
    ctx.fillText(waveforms[1].nic, x2 + 8, y2);

    ctx.fillRect(x3, y3, 4, 4);
    ctx.fillText(waveforms[2].nic, x3, y3 - 8);

}

function updateMicLocations() {
    var d12 = Number(micDistance1Input.value); // Distance between esp1 and esp2
    var d13 = Number(micDistance2Input.value); // Distance between esp1 and esp3
    var d23 = Number(micDistance3Input.value); // Distance between esp2 and esp3

    // Just put first two esps along x axis
    waveforms[0].coordinates = [0.0, 0.0];
    waveforms[1].coordinates = [d12 * 1.0, 0.0];

    var angle = Math.acos((d12 * d12 + d13 * d13 - d23 * d23) / (2 * d12 * d13));

    waveforms[2].coordinates = [
        d13 * Math.cos(angle) * 1.0,
        d13 * Math.sin(angle) * 1.0
    ];

    if (waveforms[2].coordinates[1] != 0 && connectBtn.disabled) {
        startLocalizationBtn.disabled = false;
    } else {
        startLocalizationBtn.disabled = true;
    }

    updateMicGraph();
}

function doLocalization() {
    let temperature = 21;
    let c = 331 + (0.61 * temperature);
    let mloc = [
        new Float64Array(waveforms[0].coordinates),
        new Float64Array(waveforms[1].coordinates),
        new Float64Array(waveforms[2].coordinates)
    ];

    // Allocate memory for the result_x and result_y variables
    let result_xPtr = Module._malloc(8); // double is 8 bytes
    let result_yPtr = Module._malloc(8);

    // Allocate memory for the 'mloc' array
    let mlocPtr = Module._malloc(mloc.length * mloc[0].byteLength);
    mloc.forEach((arr, index) => {
        Module.HEAPF64.set(arr, (mlocPtr / arr.BYTES_PER_ELEMENT) + (index * arr.length));
    });

    console.log(mloc);

    var start_times = [Module._malloc(8), Module._malloc(8), Module._malloc(8)];
    for (var i = 0; i < 3; i++) {
        // Assuming waveforms[i].audioBuffer.time is a BigInt representing an int64_t value
        Module.HEAP64[start_times[i] / 8] = waveforms[i].audioBuffer.time;
    }

    // Allocate memory for the array of pointers (which are int32 in Emscripten's memory)
    let start_timesPtr = Module._malloc(start_times.length * 4); // 4 bytes per pointer

    // Set the pointers in the allocated memory
    for (var i = 0; i < start_times.length; i++) {
        Module.HEAP32[start_timesPtr / 4 + i] = start_times[i];
    }

    var tracksPointers = new Uint32Array(3);
    // Allocate memory for the array of pointers (which are int32 in Emscripten's memory)
    let tracksPointersPtr = Module._malloc(tracksPointers.length * 4); // 4 bytes per int32 pointer

    // Allocate memory and get pointers
    for (let i = 0; i < 3; i++) {
        const int16Array = waveforms[i].audioBuffer.data;
        const bytesPerElement = int16Array.BYTES_PER_ELEMENT;
        const numBytes = int16Array.length * bytesPerElement;
        const ptr = Module._malloc(numBytes); // Allocate memory in the WebAssembly module
        tracksPointers[i] = ptr;
        Module.HEAP16.set(int16Array, ptr / 2); // Copy data into allocated memory
        Module.HEAPU32[tracksPointersPtr / 4 + i] = ptr;
    }

    try {
        Module._getLocationFromAudio(start_timesPtr, tracksPointersPtr, BigInt(waveforms[0].audioBuffer.data.length),
            temperature, mlocPtr, result_xPtr, result_yPtr);
    }
    catch (e) {
        console.log(e.message);
        console.log(e.stack);
    }


    let result_x = Module.HEAPF64[result_xPtr / 8];
    let result_y = Module.HEAPF64[result_yPtr / 8];
    console.log('Result X:', result_x, 'Result Y:', result_y);

    localizationResult = [result_x, result_y];
    updateMicGraph();

    // Free the allocated memory
    for (let ptr of tracksPointers) {
        Module._free(ptr);
    }
    for (let ptr of start_times) {
        Module._free(ptr);
    }
    Module._free(tracksPointersPtr);
    Module._free(start_timesPtr);
    Module._free(mlocPtr);
    Module._free(result_xPtr);
    Module._free(result_yPtr);

    for (var i = 0; i < 3; i++) {
        var aB = waveforms[i].audioBuffer;
        aB.data = undefined;
        waveforms[i].isSyncSampleTaken = false;
    }
    isLocalizationRunning = false;
}

window.connect = function connect() {

    connectBtn.disabled = !connectBtn.disabled;
    recordAllBtn.disabled = !recordAllBtn.disabled;
    checkSyncBtn.disabled = !checkSyncBtn.disabled;
    for (var i = 0; i < 3; i++) {
        waveforms[i].continueBtn.disabled = !waveforms[i].pauseBtn.disabled;
        waveforms[i].recorderBtn.disabled = false;
    }

    // start the time difference calculation
    calcTimeDifference();

    const WS_URL = 'ws:///192.168.62.46:8888'
    var ws = new WebSocket(WS_URL)
    ws.binaryType = 'arraybuffer'
    ws.addEventListener('message', function (event) {
        // Upper 32 bits of chip id as a unique identifier
        // (it may conatain the NIC part of MAC, not sure, check docs)
        var nicArray = new Uint32Array(event.data.slice(0, 4), 0, 1);
        // Time in microseconds
        var timeArray = new BigInt64Array(event.data.slice(4, 12), 0, 1);
        var buffer8Audio = event.data.slice(12);
        var int16Audio = new Int16Array(buffer8Audio);
        var nic = nicArray[0];
        var time = timeArray[0];
        var nicIndex;
        // If the NIC (chip ID) is not in the map, add it (new ESP connected)
        if (!nic_addresses.has(nic)) {
            nic_addresses.set(nic, nic_addresses.size);
            nicIndex = nic_addresses.get(nic);
            waveforms[nicIndex].nic = nic;

            var esp_id = document.getElementById("esp_id".concat(nicIndex + 1));
            esp_id.innerHTML = "ESP32S3 nr ".concat(nicIndex + 1).concat(": ").concat(nic);

            if (nic_addresses.size >= 3) {
                setUpDistanceUI();
            }
            return;
        } else {
            nicIndex = nic_addresses.get(nic);
        }
        // calculate the time difference between local machine and the esp
        local_vs_esp_time = ((performance.timeOrigin + performance.now()) * 1000 - Number(time)) / 1000;
        // set time to calculate the very
        // approximate difference between the ESP32s
        last_times[nicIndex] = time;
        // display the time difference
        waveforms[nicIndex].ESPlocalTimeDiff.innerHTML = local_vs_esp_time;
        // send the data to the worker
        waveforms[nicIndex].worker.postMessage(buffer8Audio);
        // Feed audio player data (if player is not paused)
        if (waveforms[nicIndex].continueBtn.disabled) {
            waveforms[nicIndex].player.feed(buffer8Audio);
        }
        // Feed audio to recorder
        if (waveforms[nicIndex].recorderBtn.disabled) {
            if (feedAudioToRecorder(nicIndex, int16Audio, time, waveforms[nicIndex].recordingTimeDur.value)) {
                stopAndDownloadRecording(nicIndex);
            }
        }
        if (checkSyncBtn.disabled && !waveforms[nicIndex].isSyncSampleTaken) {
            if (feedAudioToDataBuffer(nicIndex, int16Audio, time, 2)) {
                waveforms[nicIndex].isSyncSampleTaken = true;
                var allAudioReady = true;
                for (var i = 0; i < 3; i++) {
                    if (!waveforms[i].isSyncSampleTaken) {
                        allAudioReady = false;
                    }
                }
                if (allAudioReady) {
                    checkSyncBtn.disabled = false;
                    checkSync();

                }
            }
        }
        if (startLocalizationBtn.disabled && !waveforms[nicIndex].isSyncSampleTaken && isLocalizationRecording) {
            if (feedAudioToDataBuffer(nicIndex, int16Audio, time, 1)) {
                waveforms[nicIndex].isSyncSampleTaken = true;
                var allAudioReady = true;
                for (var i = 0; i < 3; i++) {
                    if (!waveforms[i].isSyncSampleTaken) {
                        allAudioReady = false;
                    }
                }
                if (allAudioReady && !isLocalizationRunning) {
                    isLocalizationRunning = true;
                    doLocalization();

                }
            }
        }
    });
}
window.changeVolume = function changeVolume(e, i) {
    waveforms[i - 1].player.volume(document.querySelector('#range'.concat(i)).value)
}
window.pause = async function pause(i) {
    await waveforms[i - 1].player.pause();
    waveforms[i - 1].pauseBtn.disabled = true;
    waveforms[i - 1].continueBtn.disabled = false;
}
window.continuePlay = function continuePlay(i) {
    waveforms[i - 1].player.continue();
    waveforms[i - 1].pauseBtn.disabled = false;
    waveforms[i - 1].continueBtn.disabled = true;
}
window.startRecording = function startRecording(i) {
    if (typeof i === 'undefined') {
        for (var j = 0; j < 3; j++) {
            waveforms[j].recorderBtn.disabled = true;
            waveforms[j].recordingTimeDur.value = document.getElementById("recordingTimeDur").value;
        }
    } else {
        waveforms[i - 1].recorderBtn.disabled = true;
    }
}
window.startCheckSync = function startCheckSync() {
    if (!isLocalizationRecording) {
        checkSyncBtn.disabled = true;
    }
}
window.startLocalization = function startLocalization() {
    if (startLocalizationBtn.disabled) {
        startLocalizationBtn.disabled = false;
        isLocalizationRecording = false;
    } else if (!checkSyncBtn.disabled || !startLocalizationBtn.disabled) {
        isLocalizationRecording = true;
        startLocalizationBtn.disabled = true;
        waveforms.forEach(function (wf, idx) {
            wf.audioBuffer.data = undefined;
            wf.isSyncSampleTaken = false;
        });
    }
}