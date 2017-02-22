'use strict'
var DOMParser = require('xmldom').DOMParser;
var fs = require('fs');
var content = fs.readFileSync(process.argv[2]).toString();

var doc = new DOMParser().parseFromString(content);

var edges = doc.getElementsByTagName('edge');

var m = {};
for(let i = 0; i < edges.length; ++i) {
	var rid = (edges[i].getAttribute('id'));
	var l = rid.split('#');
	m[l[0]] = 1;
}
console.log(Object.keys(m).length);
