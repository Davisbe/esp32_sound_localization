// worker.js
self.addEventListener('message', function (e) {
    var mean = 0;
    if (e.data.byteLength > 0) {
        var byteArray = new Int16Array(e.data);
        for (var i = 0; i < byteArray.length; ++i) {
            mean += (byteArray[i]);
        }

        mean /= 512;
        self.postMessage(mean);
    }
}); 