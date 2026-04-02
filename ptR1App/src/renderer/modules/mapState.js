// mapState.js
//เก็บข้อมูลแผนที่ที่กำลังใช้งาน (active map)

export const activeMap = {
  name: null,     // ชื่อแผนที่ที่เลือกอยู่
  base64: null,   // รูปแผนที่ในรูป base64
  meta: null      // resolution, origin (จาก YAML)
};
