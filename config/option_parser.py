import argparse
from argparse import ArgumentParser
import pandas as pd

COM_ARG_OUTPUT_FILE_NAME = 'command_line_arguments.txt'

class option_parser:
    def __init__(self):

        parser = argparse.ArgumentParser(description="main")
        #parser.add_argument("--dataset", help="name of Data")
        #parser.add_argument("--scene_value", help="number of value")
        #parser.add_argument("--Time_method", help="number of value",default=1)
        #parser.add_argument("--state_method", help="number of value",default=4)
        #parser.add_argument("--moving_method", help="number of value",default=True)
        #parser.add_argument("--Denoising", help="number of value",default=False)
        #parser.add_argument("--select_point", help="number of value",default=False)
        #parser.add_argument("--param_d", help="name of param_d",default=1)
        #parser.add_argument("--param_c", help="number of param_c",default=6)
        #parser.add_argument("--Teach_ID", help="number of value",default=2)
        #parser.add_argument("--frm", help="number of value",default=180)
        #parser.add_argument("--start", help="number of value")
        #parser.add_argument("--end", help="number of value")
        #parser.add_argument("--radar_fram", help="number of value",default=6)

        #TODO add options for sur_cam options (or related.. delete rubbish option later)
        parser = argparse.ArgumentParser(description='process some integer')
        parser.add_argument('--data_root', type=str, default="../raw_data/")
        parser.add_argument('--crop_start', help="crop start time[s]",type=int, default=10)
        parser.add_argument('--crop_end', type=int, help="crop end time[s]", default=10000000000)
        parser.add_argument('--hough_mode', type=int, default=0)
        parser.add_argument('--data_name', type=str, default="scene20") #TODO HAVE TO MODIFY
        parser.add_argument('--vis_scale', type=int, default=10)
    
        #TODO add options for sur_cam' accuracy validation
        parser.add_argument('--ref_dir', type=str, default="~/research/ref_data/")

        #tracking and data association option
        parser.add_argument('--tracker', type=str, default="IMM_KF")
        parser.add_argument('--associator', type=str, default="GNN")
        parser.add_argument('--xtss', type=str, default="Nearest")
        parser.add_argument('--evaluator', type=int, default=0)
        parser.add_argument('--tuning', type=int, default=0)
    
        self.args = parser.parse_args()
        #return args


    def get_option(self):
        return self.args
    
    def print_options(opt):
        """Print and save options
    
        It will print both current options and default values(if different).
        """
        message = ''
        message += '----------------- Options ---------------\n'
        for k, v in sorted(vars(opt).items()):
            comment = ''
            #default = parser.get_default(k)
            #if v != default:
            #    comment = '\t[default: %s]' % str(default)
            message += '{:>25}: {:<30}{}\n'.format(str(k), str(v), comment)
        message += '----------------- End -------------------'
        print(message)
    
        # save to the disk
        #xpr_dir = os.path.join(opt.checkpoints_dir, opt.name)
        #util.mkdirs(expr_dir)
        #file_name = os.path.join(expr_dir, 'options.txt')
        #with open(file_name, 'wt') as opt_file:
        #   opt_file.write(message)
        #    opt_file.write('\n')
    
    def write_options(opt, output_path):
        args_dict = vars(opt)
        for k, v in args_dict.items():
            if v == '':
                args_dict[k] = 'None'
        df = pd.DataFrame.from_dict(args_dict, orient='index').T
        df = df.sort_index(axis=1)
        #df.set_index('network_type', inplace=True)
        df.to_csv(output_path + '/' + COM_ARG_OUTPUT_FILE_NAME, sep='\t')
    
            
