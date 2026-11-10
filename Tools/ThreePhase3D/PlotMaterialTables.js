// Dependency-free SVG plot export. Usage: node PlotMaterialTables.js <tables-dir> <output.svg>
// Draws the generated data in their declared units; this is not additional validation.
const fs=require('node:fs');
const path=require('node:path');
const [directory,output]=process.argv.slice(2);
if(!directory||!output)throw Error('Expected tables directory and SVG output path.');
const liquid=JSON.parse(fs.readFileSync(path.join(directory,'liquid-101325Pa.json'),'utf8'));
const vapour=JSON.parse(fs.readFileSync(path.join(directory,'vapour-101325Pa.json'),'utf8'));
const panels=[[liquid,0,'Liquid density'],[liquid,4,'Liquid heat capacity'],[liquid,9,'Liquid dynamic viscosity'],[liquid,10,'Liquid thermal conductivity'],[liquid,1,'Liquid specific enthalpy'],[vapour,0,'Vapour density']];
const escape=s=>String(s).replaceAll('&','&amp;').replaceAll('<','&lt;');
let svg='<svg xmlns="http://www.w3.org/2000/svg" width="1100" height="1010" viewBox="0 0 1100 1010"><rect width="1100" height="1010" fill="white"/><g font-family="sans-serif" fill="#172b40"><text x="35" y="32" font-size="22">S02 water reference tables — 101325 Pa</text><text x="35" y="56" font-size="13">FP32 knots generated from the official-check-verified FP64 IAPWS reference. Separate stable phase intervals.</text>';
for(let panel=0;panel<panels.length;panel++){
    const [table,column,title]=panels[panel],x=70+(panel%2)*545,y=120+Math.floor(panel/2)*300,w=440,h=210;
    const values=Array.from({length:table.count},(_,i)=>table.values[i*12+column]);
    let low=Math.min(...values),high=Math.max(...values),padding=(high-low)*.07;low-=padding;high+=padding;
    const minT=table.minimum_temperature_k,maxT=minT+(table.count-1)*table.step_k;
    svg+=`<text x="${x}" y="${y-34}" font-size="16">${title}</text><text x="${x}" y="${y-15}" font-size="12">${escape(table.units[column])}</text>`;
    for(let i=0;i<=4;i++){
        const yy=y+h-i*h/4,number=low+(high-low)*i/4,xx=x+i*w/4;
        svg+=`<path d="M${x},${yy}H${x+w}" stroke="#e2e8ef"/><text x="${x-7}" y="${yy+4}" text-anchor="end" font-size="11">${number.toPrecision(4)}</text><text x="${xx}" y="${y+h+20}" text-anchor="middle" font-size="11">${(minT+(maxT-minT)*i/4).toFixed(1)}</text>`;
    }
    svg+=`<path d="M${x},${y}V${y+h}H${x+w}" fill="none" stroke="#172b40"/><text x="${x+w/2}" y="${y+h+40}" text-anchor="middle" font-size="12">Temperature / K</text>`;
    const points=values.map((v,i)=>`${(x+w*i/(values.length-1)).toFixed(2)},${(y+h-h*(v-low)/(high-low)).toFixed(2)}`).join(' ');
    svg+=`<polyline points="${points}" fill="none" stroke="#007a86" stroke-width="2"/>`;
}
svg+='<text x="35" y="990" font-size="12">Table domains: liquid 274–373 K; vapour 374–600 K. Gaps are excluded, not interpolated across a phase transition.</text></g></svg>';
fs.writeFileSync(output,svg);
