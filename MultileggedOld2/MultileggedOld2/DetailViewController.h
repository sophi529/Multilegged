//
//  DetailViewController.h
//  MultileggedOld
//
//  Created by Sophie Dewil on 7/20/16.
//  Copyright (c) 2016 __MyCompanyName__. All rights reserved.
//

#import <UIKit/UIKit.h>

@interface DetailViewController : UIViewController <UISplitViewControllerDelegate>

@property (strong, nonatomic) id detailItem;

@property (strong, nonatomic) IBOutlet UILabel *detailDescriptionLabel;

@end
