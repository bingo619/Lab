using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Collections;
using System.IO;

namespace LargeFilePreviewer
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
        }
        StreamReader objReader;
        string filePath;// = @"F:\FTP\Data\OSM Maps\Singapore\WA_EdgeGeometry.txt";
        int fontSize = 12;
         
        private void button1_Click(object sender, EventArgs e)
        {
            filePath = filePathTextBox.Text;
            try
            {
                if (filePath == "")
                {
                    MessageBox.Show("文件路径不能为空!");
                    return;
                }
                objReader = new StreamReader(filePath);
            }
            catch (IOException exp)
            {
                MessageBox.Show("IO exception!");
                return;
            }
            textBox.Text = "";

            //自动读10行
            int lineCount = int.Parse(lineNumberTextBox.Text);
            try
            {
                if(lineCount < 10000000)
                {
                    for (int i = 0; i < lineCount; i++)
                    {
                        string sLine = "";
                        sLine = objReader.ReadLine();
                        if (sLine == null)
                            return;
                        textBox.AppendText(sLine + "\r\n");
                    }
                }
                else
                {
                    //无用
                    string appendStrings = "";
                    for (int i = 0; i < lineCount; i++)
                    {
                        string sLine = "";
                        sLine = objReader.ReadLine();
                        if (sLine == null)
                            return;
                        appendStrings = appendStrings + sLine + "\r\n";
                        textBox.AppendText(appendStrings);
                    }
                }
            }
            catch (IOException exp)
            {
                MessageBox.Show("IO exception!");
                return;
            }
        }

        private void readButton_Click(object sender, EventArgs e)
        {
            if (objReader == null)
            {
                MessageBox.Show("请先载入文件");
                return;
            }
            int lineCount = int.Parse(lineNumberTextBox.Text);
            try
            {
                for (int i = 0; i < lineCount; i++)
                {
                    string sLine = "";
                    sLine = objReader.ReadLine();
                    if (sLine == null)
                        return;
                    textBox.AppendText(sLine + "\r\n");
                }
            }
            catch (IOException exp)
            {
                MessageBox.Show("IO exception!");
                return;
            }
        }

        private void Form1_DragEnter(object sender, DragEventArgs e)
        {
            if (e.Data.GetDataPresent(DataFormats.FileDrop))
            {
                e.Effect = DragDropEffects.Copy;
            }
            else
            {
                e.Effect = DragDropEffects.None;
            }
        }

        private void Form1_DragDrop(object sender, DragEventArgs e)
        {
            try
            {
                filePath = ((System.Array)e.Data.GetData(DataFormats.FileDrop)).GetValue(0).ToString();
                filePathTextBox.Text = filePath;
            }
            catch (Exception e1)
            {
                MessageBox.Show(e1.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            } 
        }

        private void browseButton_Click(object sender, EventArgs e)
        {
            this.openFileDialog1.ShowDialog();
            filePath = this.openFileDialog1.FileName;
            filePathTextBox.Text = filePath;
        }

        private void bigFontButton_Click(object sender, EventArgs e)
        {
            fontSize += 2;
            textBox.Font = new Font(textBox.Font.Name, fontSize);
        }

        private void smallFontButton_Click(object sender, EventArgs e)
        {
            fontSize -= 2;
            if (fontSize <= 2)
            {
                fontSize += 2;
                return;
            }
            textBox.Font = new Font(textBox.Font.Name, fontSize);
        }

        private void form1_MouseWheel(object sender, System.Windows.Forms.MouseEventArgs e)
        {
            fontSize += Convert.ToInt32(Convert.ToDouble(e.Delta) / Convert.ToDouble(60));
            if (fontSize <= 2)
            {
                fontSize = 2;
            }
            textBox.Font = new Font(textBox.Font.Name, fontSize);
        }  
    }
}
