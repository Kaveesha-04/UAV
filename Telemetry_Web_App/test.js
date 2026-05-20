const net = require('net');
const client = new net.Socket();
client.connect(5000, '127.0.0.1', () => {
    console.log('Connected');
    client.write('{"r":1.2,"p":3.4,"y":5.6,"a":10.0,"d":45.0,"glat":6.9,"glon":79.8,"gf":1,"t":1500}\n');
    setTimeout(() => client.destroy(), 1000);
});
client.on('error', console.error);
