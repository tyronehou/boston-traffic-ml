from selenium import webdriver
from selenium.webdriver.support.ui import WebDriverWait # available since 2.4.0
from selenium.webdriver.support import expected_conditions as EC # available since 2.26.0
from selenium.common.exceptions import TimeoutException
from selenium.webdriver.common.by import By
from selenium.webdriver.firefox.firefox_profile import FirefoxProfile
import os

# Scrapes traffic count data from City of Boston open data portal site into specified directory
# IMPORTANT: For the first file you have to manually approve the save location. Make sure you click the checkbox 'Do this for all future files'. Otherwise you will still have to manually save each file

class TrafficCountScraper():
    start_urls = [
        'https://sceris.cityofboston.gov/sceriswebbtd/client/SearchResultsX.aspx?SearchParams=%3CSearchParms+xmlns%3axsi%3d%22http%3a%2f%2fwww.w3.org%2f2001%2fXMLSchema-instance%22+xmlns%3axsd%3d%22http%3a%2f%2fwww.w3.org%2f2001%2fXMLSchema%22%3E%3CFolderIDs%3E%3Cint%3E56%3C%2fint%3E%3C%2fFolderIDs%3E%3CCriteria%3E%3CCr+N%3d%22Intersection+Number%22+V%3d%220%22+CompOpr%3d%22GT%22+%2f%3E%3C%2fCriteria%3E%3CUdiDisplayNamesToDisp%3E%3CD+N%3d%22Intersection+Number%22+%2f%3E%3CD+N%3d%22Associated+Streets%22+%2f%3E%3CD+N%3d%22Collection+Date%22+%2f%3E%3CD+N%3d%22Data+Type%22+%2f%3E%3CD+N%3d%22Neighborhood+Code%22+%2f%3E%3CD+N%3d%22Site+Location%22+%2f%3E%3CD+N%3d%22Street+Name%22+%2f%3E%3CD+N%3d%22Study+Period%22+%2f%3E%3CD+N%3d%22Study+Type%22+%2f%3E%3C%2fUdiDisplayNamesToDisp%3E%3C%2fSearchParms%3E'
    ]

    def __init__(self, save_dir):
        #self.driver = webdriver.Chrome()
        self.driver = webdriver.Firefox(firefox_profile=self.createProfile(save_dir))

    def createProfile(self, saveDir):
        profile = FirefoxProfile()
        profile.set_preference("browser.download.folderList",2) # save to most recent selected folder
        profile.set_preference("browser.download.manager.showWhenStarting",False)
        profile.set_preference("browser.download.dir", saveDir)
        profile.set_preference("browser.helperApps.neverAsk.saveToDisk", "application/vnd.ms-excel")
        return profile
    
    def parse(self):
        self.driver.get(self.start_urls[0])#response.url)
        wait = WebDriverWait(self.driver, 10)
        
        # View button
        viewButton = wait.until(EC.presence_of_element_located((By.ID, "Submit")))

        resultsTable = wait.until(EC.presence_of_element_located((By.ID, "ResultTable")))

        rows = resultsTable.find_elements_by_xpath('tbody/tr')
        prompted = True

        for i, row in enumerate(rows):
            checkbox = row.find_element_by_class_name('Checkbox')
            dtype = checkbox.find_element_by_name('e' + str(i)).get_attribute('value')

            print('Row {}: {}'.format(i, dtype))
            if dtype not in ['XLS', 'XLSX']:
                continue
            checkbox.find_element_by_id(str(i)).click()
            viewButton.click()
            checkbox.find_element_by_id(str(i)).click()

            if prompted:
                input('Press Enter')
                print('Not yet')
                prompted = False

    def rename(self):
        self.driver.get(self.start_urls[0])#response.url)
        wait = WebDriverWait(self.driver, 10)
        
        resultsTable = wait.until(EC.presence_of_element_located((By.ID, "ResultTable")))
        rows = resultsTable.find_elements_by_xpath('tbody/tr')

        xls_idx = 0
        xlsx_idx = 0

        for i, row in enumerate(rows):
            dtype = row.find_element_by_name('e' + str(i)).get_attribute('value')
            print('Row {}: {}'.format(i, dtype))
            if dtype not in ['XLS', 'XLSX']:
                continue

            params = []
            for label in row.find_elements_by_tag_name('p')[:2]:
                if label.text: params.append(label.text)

            params = ';'.join(params)
            params = params.replace(' ', '_')

            # rename files from NFF1 to something more informational 
            if dtype == 'XLS':
                ext = 'xls'
                idx = xls_idx
                xls_idx += 1
            elif dtype == 'XLSX':
                ext = 'xlsx'
                idx = xlsx_idx
                xlsx_idx += 1

            if idx == 0:
                old_fname = '{}/NFF1.{}'.format(d, ext)
                new_fname = '{}/{}.{}'.format(d, params, ext)
            else:
                old_fname = '{}/NFF1({}).{}'.format(d, idx, ext)
                new_fname = '{}/{}.{}'.format(d, params, ext)

            os.rename(old_fname, new_fname)

root_dir = os.path.split(os.getcwd())[0]
d = '{}/datasets/traffic_count_data/xls_data'.format(root_dir)
TrafficCountScraper(d).parse()


